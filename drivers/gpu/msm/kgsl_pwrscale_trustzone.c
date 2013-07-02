/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <mach/socinfo.h>
#include <mach/scm.h>
#include <linux/jiffies.h>

#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_device.h"

#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
#include <linux/module.h>
#endif

#define TZ_GOVERNOR_PERFORMANCE 0
#define TZ_GOVERNOR_ONDEMAND    1

#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
#define TZ_GOVERNOR_SIMPLE	3

#ifdef CONFIG_MSM_KGSL_INTERACTIVE_GOV
#define TZ_GOVERNOR_INTERACTIVE  2
#endif

#else

#ifdef CONFIG_MSM_KGSL_INTERACTIVE_GOV
#define TZ_GOVERNOR_INTERACTIVE  2

#endif
#endif

#define DEBUG 0


struct tz_priv {
	int governor;
	unsigned int no_switch_cnt;
	unsigned int skip_cnt;
	struct kgsl_power_stats bin;
};
spinlock_t tz_lock;

/* FLOOR is 5msec to capture up to 3 re-draws
 * per frame for 60fps content.
 */
#define FLOOR			5000
/* CEILING is 50msec, larger than any standard
 * frame length, but less than the idle timer.
 */
#define CEILING			50000
#define SWITCH_OFF		200
#define SWITCH_OFF_RESET_TH	40
#define SKIP_COUNTER		500
#define TZ_RESET_ID		0x3
#define TZ_UPDATE_ID		0x4

#ifdef CONFIG_MSM_SCM
/* Trap into the TrustZone, and call funcs there. */
static int __secure_tz_entry(u32 cmd, u32 val, u32 id)
{
	int ret;
	spin_lock(&tz_lock);
	__iowmb();
	ret = scm_call_atomic2(SCM_SVC_IO, cmd, val, id);
	spin_unlock(&tz_lock);
	return ret;
}
#else
static int __secure_tz_entry(u32 cmd, u32 val, u32 id)
{
	return 0;
}
#endif /* CONFIG_MSM_SCM */

#ifdef CONFIG_MSM_KGSL_INTERACTIVE_GOV
unsigned long window_time = 0;
unsigned long sample_time_ms = 100;
unsigned int up_threshold = 50;
unsigned int down_threshold = 25;

module_param(sample_time_ms, long, 0664);
module_param(up_threshold, int, 0664);
module_param(down_threshold, int, 0664);
#endif

static ssize_t tz_governor_show(struct kgsl_device *device,
				struct kgsl_pwrscale *pwrscale,
				char *buf)
{
	struct tz_priv *priv = pwrscale->priv;
	int ret;

	if (priv->governor == TZ_GOVERNOR_ONDEMAND)
		ret = snprintf(buf, 10, "ondemand\n");
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
	else if (priv->governor == TZ_GOVERNOR_SIMPLE)
		ret = snprintf(buf, 8, "simple\n");
#endif
#ifdef CONFIG_MSM_KGSL_INTERACTIVE_GOV
	else if (priv->governor == TZ_GOVERNOR_INTERACTIVE)
    		ret = snprintf(buf, 13, "interactive\n");
#endif
	else
		ret = snprintf(buf, 13, "performance\n");

	return ret;
}

static ssize_t tz_governor_store(struct kgsl_device *device,
				struct kgsl_pwrscale *pwrscale,
				 const char *buf, size_t count)
{
	char str[20];
	struct tz_priv *priv = pwrscale->priv;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int ret;

	ret = sscanf(buf, "%20s", str);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&device->mutex);

	if (!strncmp(str, "ondemand", 8))
		priv->governor = TZ_GOVERNOR_ONDEMAND;
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
	else if (!strncmp(str, "simple", 6))
		priv->governor = TZ_GOVERNOR_SIMPLE;
#endif
#ifdef CONFIG_MSM_KGSL_INTERACTIVE_GOV
	else if (!strncmp(str, "interactive", 11))
    		priv->governor = TZ_GOVERNOR_INTERACTIVE;
#endif
	else if (!strncmp(str, "performance", 11))
		priv->governor = TZ_GOVERNOR_PERFORMANCE;

	if (priv->governor == TZ_GOVERNOR_PERFORMANCE)
		kgsl_pwrctrl_pwrlevel_change(device, pwr->max_pwrlevel);

	mutex_unlock(&device->mutex);
	return count;
}

PWRSCALE_POLICY_ATTR(governor, 0644, tz_governor_show, tz_governor_store);

static struct attribute *tz_attrs[] = {
	&policy_attr_governor.attr,
	NULL
};

static struct attribute_group tz_attr_group = {
	.attrs = tz_attrs,
};

static void tz_wake(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	struct tz_priv *priv = pwrscale->priv;
	if (device->state != KGSL_STATE_NAP){
#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
		if(priv->governor == TZ_GOVERNOR_ONDEMAND ||
		 priv->governor == TZ_GOVERNOR_SIMPLE)
#else
		if(priv->governor == TZ_GOVERNOR_ONDEMAND)
#endif
		kgsl_pwrctrl_pwrlevel_change(device,
					device->pwrctrl.default_pwrlevel);
		//else if(priv->governor == TZ_GOVERNOR_PERFORMANCE)
		//	kgsl_pwrctrl_pwrlevel_change(device, device->pwrctrl.max_pwrlevel);
	}
}

#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
/* KGSL Simple GPU Governor */
/* Copyright (c) 2011-2013, Paul Reioux (Faux123). All rights reserved. */
static int default_laziness = 5;
module_param_named(simple_laziness, default_laziness, int, 0664);

static int ramp_up_threshold = 6000;
module_param_named(simple_ramp_threshold, ramp_up_threshold, int, 0664);

static int laziness;

static int simple_governor(struct kgsl_device *device, int idle_stat)
{
	int val = 0;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	/* it's currently busy */
	if (idle_stat < ramp_up_threshold) {
		if (pwr->active_pwrlevel == 0)
			val = 0; /* already maxed, so do nothing */
		else if ((pwr->active_pwrlevel > 0) &&
			(pwr->active_pwrlevel <= (pwr->num_pwrlevels - 1)))
			val = -1; /* bump up to next pwrlevel */
	/* idle case */
	} else {
		if ((pwr->active_pwrlevel >= 0) &&
			(pwr->active_pwrlevel < (pwr->num_pwrlevels - 1)))
			if (laziness > 0) {
				/* hold off for a while */
				laziness--;
				val = 0; /* don't change anything yet */
			} else {
				val = 1; /* above min, lower it */
				/* reset laziness count */
				laziness = default_laziness;
			}
		else if (pwr->active_pwrlevel == (pwr->num_pwrlevels - 1))
			val = 0; /* already @ min, so do nothing */
	}
	return val;
}
#endif

static void tz_idle(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct tz_priv *priv = pwrscale->priv;
	struct kgsl_power_stats stats;
	unsigned long total_time_ms = 0;
  	unsigned long busy_time_ms = 0;
	int val;

	/* In "performance" mode the clock speed always stays
	   the same */
	if (priv->governor == TZ_GOVERNOR_PERFORMANCE)
	{
		return;
	}

	device->ftbl->power_stats(device, &stats);
	priv->bin.total_time += stats.total_time;
	priv->bin.busy_time += stats.busy_time;
	
	if (time_is_after_jiffies(window_time + msecs_to_jiffies(sample_time_ms)))
		return;

	/* If the GPU has stayed in turbo mode for a while, *
	 * stop writing out values. */
	if (pwr->active_pwrlevel == 0) {
		if (priv->no_switch_cnt > SWITCH_OFF) {
			priv->skip_cnt++;
			if (priv->skip_cnt > SKIP_COUNTER) {
				priv->no_switch_cnt -= SWITCH_OFF_RESET_TH;
				priv->skip_cnt = 0;
			}
			return;
		}
		priv->no_switch_cnt++;
	} else {
		priv->no_switch_cnt = 0;
	}

	total_time_ms = jiffies_to_msecs((long)jiffies - (long)window_time);

  	/*
     	 * We're casting u32 here because busy_time is s64 and this would be a
         * 64-bit division and we can't do that on a 32-bit arch 
	 */
	busy_time_ms = (u32)priv->bin.busy_time / USEC_PER_MSEC;

#if DEBUG
  pr_info("GPU current load: %ld\n", busy_time_ms);
  pr_info("GPU total time load: %ld\n", total_time_ms);
  pr_info("GPU frequency: %d\n", pwr->pwrlevels[pwr->active_pwrlevel].gpu_freq);
#endif

  if ((busy_time_ms * 100) > (total_time_ms * up_threshold))
  {
    if ((pwr->active_pwrlevel > 0) &&
      (pwr->active_pwrlevel <= (pwr->num_pwrlevels - 1)))
      kgsl_pwrctrl_pwrlevel_change(device,
               pwr->active_pwrlevel - 1);
  }
  else if ((busy_time_ms * 100) < (total_time_ms * down_threshold))
  {
    if ((pwr->active_pwrlevel >= 0) &&
      (pwr->active_pwrlevel < (pwr->num_pwrlevels - 1)))
      kgsl_pwrctrl_pwrlevel_change(device,
               pwr->active_pwrlevel + 1);
		#ifdef CONFIG_MSM_KGSL_SIMPLE_GOV
		if (priv->governor == TZ_GOVERNOR_SIMPLE)
		val = simple_governor(device, total_time_ms-busy_time_ms);
		else
		val = __secure_tz_entry(TZ_UPDATE_ID, total_time_ms-busy_time_ms, device->id);
		#else
		val = __secure_tz_entry(TZ_UPDATE_ID, total_time_ms-busy_time_ms, device->id);
		#endif
	}
	priv->bin.total_time = 0;
	priv->bin.busy_time = 0;
	window_time = jiffies;
	if (val) {
		kgsl_pwrctrl_pwrlevel_change(device,
					     pwr->active_pwrlevel + val);
	}
}

static void tz_busy(struct kgsl_device *device,
	struct kgsl_pwrscale *pwrscale)
{
	device->on_time = ktime_to_us(ktime_get());
}

static void tz_sleep(struct kgsl_device *device,
	struct kgsl_pwrscale *pwrscale)
{
	struct tz_priv *priv = pwrscale->priv;

  kgsl_pwrctrl_pwrlevel_change(device, 3);
  priv->bin.total_time = 0;
  priv->bin.busy_time = 0;
  window_time = jiffies;
}

#ifdef CONFIG_MSM_SCM
static int tz_init(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	struct tz_priv *priv;

	priv = pwrscale->priv = kzalloc(sizeof(struct tz_priv), GFP_KERNEL);
	if (pwrscale->priv == NULL)
		return -ENOMEM;

	priv->governor = TZ_GOVERNOR_INTERACTIVE;
	spin_lock_init(&tz_lock);
	kgsl_pwrscale_policy_add_files(device, pwrscale, &tz_attr_group);

	return 0;
}
#else
static int tz_init(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	return -EINVAL;
}
#endif /* CONFIG_MSM_SCM */

static void tz_close(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
{
	kgsl_pwrscale_policy_remove_files(device, pwrscale, &tz_attr_group);
	kfree(pwrscale->priv);
	pwrscale->priv = NULL;
}

struct kgsl_pwrscale_policy kgsl_pwrscale_policy_tz = {
	.name = "trustzone",
	.init = tz_init,
	.busy = tz_busy,
	.idle = tz_idle,
	.sleep = tz_sleep,
	.wake = tz_wake,
	.close = tz_close
};
EXPORT_SYMBOL(kgsl_pwrscale_policy_tz);
