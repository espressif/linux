// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif pinctrl core: SoC-independent skeleton.
 *
 * Owns region mapping, DT parsing, map generation, and all register
 * programming. Each SoC (e.g. pinctrl-esp32s31.c) only describes
 * its hardware as data (regions + iomux layers) plus a few thin hooks.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/soc/espressif/esp-gpio.h>
#include "pinctrl-esp.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"

/* ---------------------------------------------------------------------
 * MMIO helpers
 * ---------------------------------------------------------------------
 */
void esp_reg_update(void __iomem *reg, u32 mask, u32 value,
		    raw_spinlock_t *lock)
{
	unsigned long flags;
	u32 v;

	raw_spin_lock_irqsave(lock, flags);
	v = readl(reg);
	v = (v & ~mask) | (value & mask);
	writel(v, reg);
	raw_spin_unlock_irqrestore(lock, flags);
}

/* Region access: NULL-safe wrappers around the per-region MMIO mappings. */
static u32 esp_region_read(struct esp_pinctrl *epctl, enum esp_regmap_id id,
			   u32 off)
{
	if (epctl->regs[id])
		return readl(epctl->regs[id] + off);
	return 0;
}

void esp_region_write(struct esp_pinctrl *epctl, enum esp_regmap_id id,
		      u32 off, u32 val)
{
	if (epctl->regs[id])
		writel(val, epctl->regs[id] + off);
}

void esp_region_update(struct esp_pinctrl *epctl, enum esp_regmap_id id,
		       u32 off, u32 mask, u32 val)
{
	if (epctl->regs[id])
		esp_reg_update(epctl->regs[id] + off, mask, val, &epctl->lock);
}

/* Per-pin register offset within a layer's region. */
static u32 esp_layer_off(const struct esp_mux_layer *l, unsigned int pin)
{
	u32 stride = l->reg_stride ? l->reg_stride : 4;

	return l->reg_base + (pin - l->first_pin) * stride;
}

/* True if the layer exists and covers this pin. */
static bool esp_layer_covers(struct esp_pinctrl *epctl,
			     const struct esp_mux_layer *l, unsigned int pin)
{
	return l->mask && pin >= l->first_pin && epctl->regs[l->map];
}

/* Place val (right-justified) into the layer's field for pin. */
static void esp_layer_write(struct esp_pinctrl *epctl,
			    const struct esp_mux_layer *l, unsigned int pin,
			    u32 val)
{
	if (!esp_layer_covers(epctl, l, pin))
		return;
	esp_region_update(epctl, l->map, esp_layer_off(l, pin), l->mask,
			  (val << __ffs(l->mask)) & l->mask);
}

/* ---------------------------------------------------------------------
 * Pad owners: blocks that hold pads away from the IO_MUX (see the struct).
 * ---------------------------------------------------------------------
 */
/* The owner whose range covers pin and whose region is mapped, or NULL. */
static const struct esp_pad_owner *esp_owner_of(struct esp_pinctrl *epctl,
						unsigned int pin)
{
	const struct esp_pinctrl_soc *soc = epctl->soc;
	unsigned int i;

	for (i = 0; i < soc->nowners; i++) {
		const struct esp_pad_owner *o = &soc->owners[i];

		if (pin >= o->first_pin && pin <= o->last_pin &&
		    (epctl->regs[o->map] || epctl->syscons[o->map]))
			return o;
	}

	return NULL;
}

/* The owner a function hands its pads to, or NULL for an ordinary function. */
static const struct esp_pad_owner *esp_fn_owner(const struct esp_pinctrl_soc *soc,
						const struct esp_pin_function *fn)
{
	return fn->owner ? &soc->owners[fn->owner - 1] : NULL;
}

static bool esp_owner_holds(struct esp_pinctrl *epctl,
			    const struct esp_pad_owner *o)
{
	unsigned int val = 0;

	if (epctl->syscons[o->map])
		regmap_read(epctl->syscons[o->map], o->reg, &val);
	else
		val = esp_region_read(epctl, o->map, o->reg);

	return val & o->mask;
}

static void esp_owner_set(struct esp_pinctrl *epctl,
			  const struct esp_pad_owner *o, bool held)
{
	u32 val = held ? o->mask : 0;

	if (epctl->syscons[o->map])
		regmap_update_bits(epctl->syscons[o->map], o->reg, o->mask, val);
	else
		esp_region_update(epctl, o->map, o->reg, o->mask, val);
}

/*
 * The layer set governing a pin's electrical config: an owner's own layers
 * while it holds the pin, the IO_MUX layers otherwise.
 */
static const struct esp_mux_layer *esp_layers_for(struct esp_pinctrl *epctl,
						  unsigned int pin)
{
	const struct esp_pad_owner *o = esp_owner_of(epctl, pin);

	if (o && o->layers && esp_owner_holds(epctl, o))
		return o->layers;

	return epctl->soc->layers;
}

/*
 * The only place a pad's IO_MUX function is written. A pad another block holds
 * is released first, or the function written would not reach it.
 */
void esp_set_mux(struct esp_pinctrl *epctl, unsigned int pin, u32 mode)
{
	const struct esp_pad_owner *o = esp_owner_of(epctl, pin);

	if (o)
		esp_owner_set(epctl, o, false);

	esp_layer_write(epctl, &epctl->soc->layers[ESP_FIELD_FUNC], pin, mode);
}

/* Read the layer's field for pin, right-justified (0 if the region is absent). */
static u32 esp_layer_get(struct esp_pinctrl *epctl,
			 const struct esp_mux_layer *l, unsigned int pin)
{
	if (!esp_layer_covers(epctl, l, pin))
		return 0;
	return (esp_region_read(epctl, l->map, esp_layer_off(l, pin)) & l->mask) >>
	       __ffs(l->mask);
}

static void esp_open_drain_write(struct esp_pinctrl *epctl, unsigned int pin,
				 u32 val)
{
	const struct esp_mux_layer *l =
		&esp_layers_for(epctl, pin)[ESP_FIELD_OPEN_DRAIN];
	void __iomem *reg;
	unsigned long flags;
	u32 v;

	if (!esp_layer_covers(epctl, l, pin))
		return;

	reg = epctl->regs[l->map] + esp_layer_off(l, pin);
	raw_spin_lock_irqsave(&esp_gpio_pin_reg_lock, flags);
	v = readl(reg);
	v = (v & ~l->mask) | ((val << __ffs(l->mask)) & l->mask);
	writel(v, reg);
	raw_spin_unlock_irqrestore(&esp_gpio_pin_reg_lock, flags);
}

static u32 esp_open_drain_get(struct esp_pinctrl *epctl, unsigned int pin)
{
	const struct esp_mux_layer *l =
		&esp_layers_for(epctl, pin)[ESP_FIELD_OPEN_DRAIN];

	if (!esp_layer_covers(epctl, l, pin))
		return 0;
	return (esp_region_read(epctl, l->map, esp_layer_off(l, pin)) & l->mask) >>
	       __ffs(l->mask);
}

/* ---------------------------------------------------------------------
 * Region mapping: map every region the SoC declares into regs[id].
 * ---------------------------------------------------------------------
 */
/*
 * A region another driver has already claimed is mapped without a claim of our
 * own; devm_ioremap_resource() would take the reservation and fail with -EBUSY.
 */
static void __iomem *esp_ioremap_region(struct device *dev, struct resource *res,
					bool shared)
{
	void __iomem *base;

	if (!shared)
		return devm_ioremap_resource(dev, res);

	base = devm_ioremap(dev, res->start, resource_size(res));

	return base ? base : ERR_PTR(-ENOMEM);
}

static int esp_map_region(struct platform_device *pdev,
			  struct esp_pinctrl *epctl,
			  const struct esp_region *region)
{
	struct device *dev = &pdev->dev;
	void __iomem *base;

	if (region->syscon) {
		struct regmap *map;

		map = syscon_regmap_lookup_by_phandle_optional(dev->of_node,
							       region->syscon);
		if (IS_ERR(map))
			return dev_err_probe(dev, PTR_ERR(map),
					     "cannot get '%s' syscon\n",
					     region->syscon);
		if (!map && !region->optional)
			return dev_err_probe(dev, -ENODEV, "missing '%s'\n",
					     region->syscon);

		epctl->syscons[region->id] = map;
		return 0;
	}

	if (region->dt_name) {
		struct resource *res;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   region->dt_name);
		if (!res) {
			if (region->optional)
				return 0;
			return dev_err_probe(dev, -ENODEV,
					     "missing '%s' region\n",
					     region->dt_name);
		}
		base = esp_ioremap_region(dev, res, region->shared);
	} else {
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM,
					    region->reg_index);
		if (!res) {
			if (region->optional)
				return 0;
			return dev_err_probe(dev, -ENODEV,
					     "missing region %d\n", region->id);
		}
		base = esp_ioremap_region(dev, res, region->shared);
	}

	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				     "cannot map region %d\n", region->id);

	epctl->regs[region->id] = base;
	return 0;
}

static int esp_map_regions(struct platform_device *pdev,
			   struct esp_pinctrl *epctl)
{
	const struct esp_pinctrl_soc *soc = epctl->soc;
	unsigned int i;
	int ret;

	for (i = 0; i < soc->nregions; i++) {
		const struct esp_region *region = &soc->regions[i];

		if (region->id >= ESP_REGMAP_MAX) {
			dev_err(&pdev->dev, "bad region id %d\n", region->id);
			return -EINVAL;
		}
		ret = esp_map_region(pdev, epctl, region);
		if (ret)
			return ret;
	}
	return 0;
}

/* ---------------------------------------------------------------------
 * pinctrl_ops: group lookup and DT-to-map translation
 * ---------------------------------------------------------------------
 */
static const struct group_desc *
esp_find_group_by_name(struct pinctrl_dev *pctldev, const char *name)
{
	const struct group_desc *grp;
	unsigned int i;

	for (i = 0; i < pctldev->num_groups; i++) {
		grp = pinctrl_generic_get_group(pctldev, i);
		if (grp && !strcmp(grp->grp.name, name))
			return grp;
	}
	return NULL;
}

static int esp_dt_node_to_map(struct pinctrl_dev *pctldev,
			      struct device_node *np,
			      struct pinctrl_map **map,
			      unsigned int *num_maps)
{
	struct esp_pinctrl *epctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int reserved_maps = 0;
	unsigned long *configs = NULL;
	unsigned int nconfigs = 0;
	struct device_node *parent;
	const struct group_desc *grp;
	unsigned int i;
	int ret;

	grp = esp_find_group_by_name(pctldev, np->name);
	if (!grp) {
		dev_err(epctl->dev, "no group for node %pOF\n", np);
		return -EINVAL;
	}

	*map = NULL;
	*num_maps = 0;

	parent = of_get_parent(np);
	if (!parent)
		return -EINVAL;

	ret = pinconf_generic_parse_dt_config(np, pctldev, &configs, &nconfigs);
	if (ret)
		goto out;

	/* One mux entry, plus one pinconf entry per pin when configs exist. */
	ret = pinctrl_utils_reserve_map(pctldev, map, &reserved_maps, num_maps,
					1 + (nconfigs ? grp->grp.npins : 0));
	if (ret)
		goto out;
	ret = pinctrl_utils_add_map_mux(pctldev, map, &reserved_maps, num_maps,
					np->name, parent->name);
	if (ret)
		goto out;

	for (i = 0; nconfigs && i < grp->grp.npins; i++) {
		unsigned int pin = ((struct esp_pin_cfg *)grp->data)[i].pin;

		ret = pinctrl_utils_add_map_configs(pctldev, map, &reserved_maps,
						    num_maps,
						    pin_get_name(pctldev, pin),
						    configs, nconfigs,
						    PIN_MAP_TYPE_CONFIGS_PIN);
		if (ret)
			goto out;
	}

out:
	kfree(configs);
	of_node_put(parent);
	if (ret)
		pinctrl_utils_free_map(pctldev, *map, *num_maps);
	return ret;
}

static const struct pinctrl_ops esp_pctrl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name   = pinctrl_generic_get_group_name,
	.get_group_pins   = pinctrl_generic_get_group_pins,
	.dt_node_to_map   = esp_dt_node_to_map,
	.dt_free_map      = pinctrl_utils_free_map,
};

/* ------------------------------------------------------------------------
 * pinmux: distribute each pin's function across the mux layers and matrix.
 * ------------------------------------------------------------------------
 */
/*
 * The pin's function id selects a full mux path from the SoC function table,
 * distributed across the mux layer (IO_MUX selector) and the GPIO matrix.
 */
static int esp_apply_function(struct esp_pinctrl *epctl, struct esp_pin_cfg *cfg)
{
	const struct esp_pinctrl_soc *soc = epctl->soc;
	const struct esp_pin_function *fn;
	const struct esp_pad_owner *o;
	int ret;

	if (cfg->func >= soc->nfunctions) {
		dev_err(epctl->dev, "pin %u: function %u out of range\n",
			cfg->pin, cfg->func);
		return -EINVAL;
	}
	fn = &soc->functions[cfg->func];

	if (fn->matrix_in >= 0 && soc->matrix_set_in) {
		ret = soc->matrix_set_in(epctl, cfg->pin, fn->matrix_in);
		if (ret)
			return ret;
	}
	if (fn->matrix_out >= 0 && soc->matrix_set_out) {
		ret = soc->matrix_set_out(epctl, cfg->pin, fn->matrix_out);
		if (ret)
			return ret;
	}

	/*
	 * A function that names this pad's owner hands the pad to that block,
	 * and the IO_MUX function is then irrelevant. On any other pad the
	 * function is an ordinary IO_MUX one, which releases the pad first.
	 */
	o = esp_owner_of(epctl, cfg->pin);
	if (o && o == esp_fn_owner(soc, fn))
		esp_owner_set(epctl, o, true);
	else if (fn->top_mux >= 0)
		esp_set_mux(epctl, cfg->pin, fn->top_mux);

	dev_dbg(epctl->dev, "%s func: pin %u func %u top=%d\n",
		soc->name, cfg->pin, cfg->func, fn->top_mux);
	return 0;
}

static int esp_pinmux_set(struct pinctrl_dev *pctldev,
		       unsigned int selector, unsigned int group)
{
	struct esp_pinctrl *epctl = pinctrl_dev_get_drvdata(pctldev);
	const struct function_desc *func;
	struct group_desc *grp;
	unsigned int i;
	int ret;

	grp  = pinctrl_generic_get_group(pctldev, group);
	func = pinmux_generic_get_function(pctldev, selector);
	if (!grp || !func)
		return -EINVAL;

	for (i = 0; i < grp->grp.npins; i++) {
		struct esp_pin_cfg *cfg = &((struct esp_pin_cfg *)grp->data)[i];

		ret = esp_apply_function(epctl, cfg);
		if (ret)
			return ret;
	}

	return 0;
}

static int esp_pinmux_gpio_request_enable(struct pinctrl_dev *pctldev,
				       struct pinctrl_gpio_range *range,
				       unsigned int pin)
{
	struct esp_pinctrl *epctl = pinctrl_dev_get_drvdata(pctldev);
	const struct esp_mux_layer *layers = epctl->soc->layers;
	int ret;

	if (pin >= epctl->soc->pin_count)
		return -EINVAL;

	if (epctl->soc->gpio_request) {
		ret = epctl->soc->gpio_request(epctl, pin);
		if (ret)
			return ret;
	} else {
		esp_set_mux(epctl, pin, 0);
	}

	esp_layer_write(epctl, &layers[ESP_FIELD_IE], pin, 1);
	return 0;
}

static void esp_pinmux_gpio_disable_free(struct pinctrl_dev *pctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int pin)
{
	struct esp_pinctrl *epctl = pinctrl_dev_get_drvdata(pctldev);
	const struct esp_mux_layer *layers = epctl->soc->layers;

	if (pin >= epctl->soc->pin_count)
		return;

	esp_layer_write(epctl, &layers[ESP_FIELD_IE], pin, 0);
}

static int esp_pinmux_gpio_set_direction(struct pinctrl_dev *pctldev,
				      struct pinctrl_gpio_range *range,
				      unsigned int pin, bool input)
{
	struct esp_pinctrl *epctl = pinctrl_dev_get_drvdata(pctldev);
	const struct esp_mux_layer *layers = epctl->soc->layers;

	if (pin >= epctl->soc->pin_count)
		return -EINVAL;

	esp_layer_write(epctl, &layers[ESP_FIELD_IE], pin, input ? 1 : 0);
	return 0;
}

static const struct pinmux_ops esp_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name   = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux	     = esp_pinmux_set,
	.gpio_request_enable = esp_pinmux_gpio_request_enable,
	.gpio_disable_free   = esp_pinmux_gpio_disable_free,
	.gpio_set_direction  = esp_pinmux_gpio_set_direction,
	.strict		     = true,
};

/* Defined here rather than in gpio-esp32s31, which depends on this driver. */
DEFINE_RAW_SPINLOCK(esp_gpio_pin_reg_lock);
EXPORT_SYMBOL_GPL(esp_gpio_pin_reg_lock);

/* ---------------------------------------------------------------------
 * pinconf: standard pinconf-generic parameters mapped to the pad layers. Only
 * the properties a group actually declares are written, so undeclared fields
 * keep their reset/boot value.
 * ---------------------------------------------------------------------
 */

/* Translate a register FUN_DRV encoding to milliamps using the SoC's table. */
static u32 esp_drv_reg_to_mA(const struct esp_pinctrl_soc *soc, u32 reg_val)
{
	if (!soc->drv_mA || reg_val >= soc->ndrv_levels)
		return reg_val;
	return soc->drv_mA[reg_val];
}

/* Translate a milliamp DT argument to its FUN_DRV level; only listed values exist. */
static int esp_drv_mA_to_reg(const struct esp_pinctrl_soc *soc, u32 mA)
{
	unsigned int i;

	if (!soc->drv_mA)
		return mA;

	for (i = 0; i < soc->ndrv_levels; i++) {
		if (soc->drv_mA[i] == mA)
			return i;
	}
	return -EINVAL;
}

static int esp_pinconf_get(struct pinctrl_dev *pctldev,
			   unsigned int pin, unsigned long *config)
{
	struct esp_pinctrl *epctl = pinctrl_dev_get_drvdata(pctldev);
	const struct esp_mux_layer *layers = esp_layers_for(epctl, pin);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg;

	if (pin >= epctl->soc->pin_count)
		return -EINVAL;

	switch (param) {
	case PIN_CONFIG_INPUT_ENABLE:
		arg = esp_layer_get(epctl, &layers[ESP_FIELD_IE], pin);
		if (!arg)
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		arg = esp_layer_get(epctl, &layers[ESP_FIELD_PULL_UP], pin);
		if (!arg)
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = esp_layer_get(epctl, &layers[ESP_FIELD_PULL_DOWN], pin);
		if (!arg)
			return -EINVAL;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		arg = esp_open_drain_get(epctl, pin);
		if (!arg)
			return -EINVAL;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = esp_layer_get(epctl, &layers[ESP_FIELD_DRIVE], pin);
		arg = esp_drv_reg_to_mA(epctl->soc, arg);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int esp_pinconf_set(struct pinctrl_dev *pctldev,
			   unsigned int pin, unsigned long *configs,
			   unsigned int num_configs)
{
	struct esp_pinctrl *epctl = pinctrl_dev_get_drvdata(pctldev);
	const struct esp_mux_layer *layers = esp_layers_for(epctl, pin);
	unsigned int i;
	int drv;

	if (pin >= epctl->soc->pin_count)
		return -EINVAL;

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_INPUT_ENABLE:
			esp_layer_write(epctl, &layers[ESP_FIELD_IE], pin, !!arg);
			break;
		case PIN_CONFIG_BIAS_DISABLE:
			esp_layer_write(epctl, &layers[ESP_FIELD_PULL_UP], pin, 0);
			esp_layer_write(epctl, &layers[ESP_FIELD_PULL_DOWN], pin, 0);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			esp_layer_write(epctl, &layers[ESP_FIELD_PULL_UP], pin, 1);
			esp_layer_write(epctl, &layers[ESP_FIELD_PULL_DOWN], pin, 0);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			esp_layer_write(epctl, &layers[ESP_FIELD_PULL_DOWN], pin, 1);
			esp_layer_write(epctl, &layers[ESP_FIELD_PULL_UP], pin, 0);
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			esp_open_drain_write(epctl, pin, 1);
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			esp_open_drain_write(epctl, pin, 0);
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			drv = esp_drv_mA_to_reg(epctl->soc, arg);
			if (drv < 0) {
				dev_err(epctl->dev,
					"pin %u: unsupported drive-strength %u mA\n",
					pin, arg);
				return drv;
			}
			esp_layer_write(epctl, &layers[ESP_FIELD_DRIVE], pin, drv);
			break;
		default:
			return -ENOTSUPP;
		}
	}

	return 0;
}

static const struct pinconf_ops esp_pinconf_ops = {
	.is_generic     = true,
	.pin_config_get = esp_pinconf_get,
	.pin_config_set = esp_pinconf_set,
};

/* ---------------------------------------------------------------------
 * DT parsing skeleton
 * ---------------------------------------------------------------------
 */
/*
 * Each group node lists its pins as ESP_PINMUX(pin, func) cells; pad electrical
 * config comes from standard pinconf-generic properties handled at map time.
 */
static int esp_parse_group(struct device_node *np, struct esp_pinctrl *epctl)
{
	struct esp_pin_cfg *cfgs = NULL;
	unsigned int *pins = NULL;
	int npins, i, ret;

	npins = of_property_count_u32_elems(np, "pinmux");
	if (npins <= 0) {
		/* Accept empty scaffolding groups (no MMIO programming). */
		dev_dbg(epctl->dev, "empty group %pOF\n", np);
		npins = 0;
	} else {
		cfgs = devm_kcalloc(epctl->dev, npins, sizeof(*cfgs), GFP_KERNEL);
		pins = devm_kcalloc(epctl->dev, npins, sizeof(*pins), GFP_KERNEL);
		if (!cfgs || !pins)
			return -ENOMEM;
	}

	for (i = 0; i < npins; i++) {
		u32 v;

		ret = of_property_read_u32_index(np, "pinmux", i, &v);
		if (ret)
			return ret;

		cfgs[i].pin  = ESP_PINMUX_GET_PIN(v);
		cfgs[i].func = ESP_PINMUX_GET_FUNC(v);
		if (cfgs[i].pin >= epctl->soc->pin_count) {
			dev_err(epctl->dev, "%pOF: pin %u out of range (0..%u)\n",
				np, cfgs[i].pin, epctl->soc->pin_count - 1);
			return -EINVAL;
		}
		pins[i] = cfgs[i].pin;
	}

	/*
	 * The group is published complete, and by the core, which owns the
	 * group tree's lock and the group count.
	 */
	ret = pinctrl_generic_add_group(epctl->pctl, np->name, pins, npins, cfgs);
	if (ret < 0)
		return dev_err_probe(epctl->dev, ret, "cannot add group %pOF\n",
				     np);

	return 0;
}

static int esp_parse_function(struct device_node *np, struct esp_pinctrl *epctl)
{
	struct pinctrl_dev *pctl = epctl->pctl;
	struct device_node *child;
	const char **group_names;
	unsigned int ngroups, i;
	int ret;

	ngroups = of_get_child_count(np);
	if (!ngroups) {
		dev_err(epctl->dev, "function %pOF has no groups\n", np);
		return -EINVAL;
	}

	group_names = devm_kcalloc(epctl->dev, ngroups, sizeof(*group_names),
				   GFP_KERNEL);
	if (!group_names)
		return -ENOMEM;

	i = 0;
	for_each_child_of_node(np, child)
		group_names[i++] = child->name;

	ret = pinmux_generic_add_function(pctl, np->name, group_names,
					  ngroups, NULL);
	if (ret < 0) {
		dev_err(epctl->dev, "add function %pOF: %d\n", np, ret);
		return ret;
	}

	for_each_child_of_node(np, child) {
		ret = esp_parse_group(child, epctl);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	return 0;
}

static int esp_parse_dt(struct platform_device *pdev, struct esp_pinctrl *epctl)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	unsigned int nfuncs;

	if (!np)
		return -ENODEV;

	nfuncs = of_get_child_count(np);
	if (!nfuncs) {
		dev_err(&pdev->dev, "no pinctrl functions defined in DT\n");
		return -EINVAL;
	}

	for_each_child_of_node(np, child) {
		int ret = esp_parse_function(child, epctl);

		if (ret) {
			of_node_put(child);
			return ret;
		}
	}
	return 0;
}

/* ---------------------------------------------------------------------
 * probe
 * ---------------------------------------------------------------------
 */
/*
 * Both are optional, and absent on a SoC whose iomux needs no gating: a node
 * with no clocks yields a bulk of zero and no reset control. The reset is only
 * ever deasserted -- asserting it would return every pad to its reset function,
 * the console UART's included.
 */
static int esp_prepare_clocks(struct device *dev)
{
	struct reset_control *rst;
	struct clk_bulk_data *clks;
	int ret;

	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot enable clocks\n");

	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "cannot get reset\n");

	ret = reset_control_deassert(rst);
	if (ret)
		return dev_err_probe(dev, ret, "cannot deassert reset\n");

	return 0;
}

static int esp_pinctrl_probe(struct platform_device *pdev)
{
	const struct esp_pinctrl_soc *soc;
	struct esp_pinctrl *epctl;
	struct pinctrl_desc *desc;
	int ret;

	soc = of_device_get_match_data(&pdev->dev);
	if (!soc)
		return -ENODEV;

	epctl = devm_kzalloc(&pdev->dev, sizeof(*epctl), GFP_KERNEL);
	if (!epctl)
		return -ENOMEM;

	epctl->dev = &pdev->dev;
	epctl->soc = soc;
	raw_spin_lock_init(&epctl->lock);

	ret = esp_prepare_clocks(&pdev->dev);
	if (ret)
		return ret;

	ret = esp_map_regions(pdev, epctl);
	if (ret)
		return ret;

	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->name    = dev_name(&pdev->dev);
	desc->pins    = soc->pins;
	desc->npins   = soc->npins;
	desc->pctlops = &esp_pctrl_ops;
	desc->pmxops  = &esp_pinmux_ops;
	desc->confops = &esp_pinconf_ops;
	desc->owner   = THIS_MODULE;

	platform_set_drvdata(pdev, epctl);

	ret = devm_pinctrl_register_and_init(&pdev->dev, desc, epctl,
					     &epctl->pctl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "cannot register pinctrl\n");

	ret = esp_parse_dt(pdev, epctl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to parse pinctrl DT\n");

	ret = pinctrl_enable(epctl->pctl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "cannot enable pinctrl\n");

	dev_info(&pdev->dev, "%s pinctrl ready: %u pins\n",
		 soc->name, soc->pin_count);
	return 0;
}

static const struct of_device_id esp_pinctrl_of_match[] = {
	{ .compatible = "esp,esp32s31-pinctrl", .data = &esp32s31_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, esp_pinctrl_of_match);

static struct platform_driver esp_pinctrl_driver = {
	.probe  = esp_pinctrl_probe,
	.driver = {
		.name		     = "pinctrl",
		.of_match_table      = esp_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init esp_pinctrl_init(void)
{
	return platform_driver_register(&esp_pinctrl_driver);
}
arch_initcall(esp_pinctrl_init);

MODULE_AUTHOR("Espressif Systems");
MODULE_DESCRIPTION("Espressif pinctrl driver");
MODULE_LICENSE("GPL");
