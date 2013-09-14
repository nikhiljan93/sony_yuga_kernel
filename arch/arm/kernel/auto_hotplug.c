/* Copyright (c) 2012, Will Tisdale <willtisdale@gmail.com>. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * Generic auto hotplug driver for ARM SoCs. Targeted at current generation
 * SoCs with dual and quad core applications processors.
 * Automatically hotplugs online and offline CPUs based on system load.
 * It is also capable of immediately onlining a core based on an external
 * event by calling void hotplug_boostpulse(void)
 *
 * Not recommended for use with OMAP4460 due to the potential for lockups
 * whilst hotplugging.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>
#include <mach/cpufreq.h>
#endif

/*
 * Enable debug output to dump the average
 * calculations and ring buffer array values
 * WARNING: Enabling this causes a ton of overhead
 *
 * FIXME: Turn it into debugfs stats (somehow)
 * because currently it is a sack of shit.
 */
#define DEBUG 0

#define CPUS_AVAILABLE		num_possible_cpus()
/*
 * SAMPLING_PERIODS * MIN_SAMPLING_RATE is the minimum
 * load history which will be averaged
 */
#define SAMPLING_PERIODS	8
#define INDEX_MAX_VALUE		(SAMPLING_PERIODS - 1)
#define AVG_NR(nr)			(nr >> 3)
/*
 * MIN_SAMPLING_RATE is scaled based on num_online_cpus()
 */
#define DEFAULT_SAMPLING_RATE	20

/*
 * Load defines:
 * ENABLE_ALL is a high watermark to rapidly online all CPUs
 *
 * ENABLE is the load which is required to enable 1 extra CPU
 * DISABLE is the load at which a CPU is disabled
 * These two are scaled based on num_online_cpus()
 */
#define ENABLE_ALL_LOAD_THRESHOLD	(120 * CPUS_AVAILABLE)
#define ENABLE_LOAD_THRESHOLD		200
#define DISABLE_LOAD_THRESHOLD		50

/* Control flags */
unsigned char flags;
#define HOTPLUG_DISABLED	(1 << 0)
#define HOTPLUG_PAUSED		(1 << 1)
#define BOOSTPULSE_ACTIVE	(1 << 2)
#define EARLYSUSPEND_ACTIVE	(1 << 3)

struct delayed_work hotplug_decision_work;
struct delayed_work hotplug_unpause_work;
struct work_struct hotplug_online_all_work;
struct work_struct hotplug_online_single_work;
struct delayed_work hotplug_offline_work;
struct work_struct hotplug_offline_all_work;
struct work_struct hotplug_boost_online_work;

static unsigned int history[SAMPLING_PERIODS];
static unsigned int index;

static int enabled = 0;
static unsigned int min_online_cpus = 2;
static unsigned int max_online_cpus = 4;
static unsigned int min_sampling_rate_ms = DEFAULT_SAMPLING_RATE;
static unsigned int min_sampling_rate = 0;
static unsigned int sampling_rate_scale = 2;
static unsigned int enable_load_threshold = ENABLE_LOAD_THRESHOLD;
static unsigned int disable_load_threshold = DISABLE_LOAD_THRESHOLD;

static unsigned int screen_off_max_freq = CONFIG_MSM_CPU_FREQ_SUSPEND_MAX;
static unsigned int screen_off_min_freq = CONFIG_MSM_CPU_FREQ_SUSPEND_MIN;

void hotplug_disable(bool flag);

#ifdef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
static unsigned int cpufreq_save_min;
static unsigned int cpufreq_save_max;
static unsigned int scaling_save_min;
static unsigned int scaling_save_max;
#endif

void hotplug_disable(bool flag);

static void auto_hotplug_early_suspend(struct early_suspend *handler)
{
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	// Set min/max only cpu0, and offline other.
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);

	cpufreq_save_min = policy->cpuinfo.min_freq;
	cpufreq_save_max = policy->cpuinfo.max_freq;
	scaling_save_min = policy->min;
	scaling_save_max = policy->max;

	policy->cpuinfo.min_freq = screen_off_min_freq;
	policy->cpuinfo.max_freq = screen_off_max_freq;
	policy->min = screen_off_min_freq;
	policy->max = screen_off_max_freq;

	cpufreq_cpu_put(policy);
#endif

#if DEBUG
	pr_info("auto_hotplug: early suspend handler\n");
#endif
	flags |= EARLYSUSPEND_ACTIVE;

	/* Cancel all scheduled delayed work to avoid races */
	cancel_delayed_work_sync(&hotplug_offline_work);
	cancel_delayed_work_sync(&hotplug_decision_work);
	if (num_online_cpus() > 1) {
#if DEBUG
		pr_info("auto_hotplug: Offlining CPUs for early suspend\n");
#endif
		schedule_work_on(0, &hotplug_offline_all_work);
	}
}

static void auto_hotplug_late_resume(struct early_suspend *handler)
{
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	// Restore cpu0 min/max freq
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	policy->cpuinfo.min_freq = cpufreq_save_min;
	policy->cpuinfo.max_freq = cpufreq_save_max;
	policy->min = scaling_save_min;
	policy->max = scaling_save_max;
	cpufreq_cpu_put(policy);
#endif

#if DEBUG
	pr_info("auto_hotplug: late resume handler\n");
#endif
	flags &= ~EARLYSUSPEND_ACTIVE;

	if(!enabled)
		schedule_work(&hotplug_online_all_work);
    else
    	schedule_delayed_work_on(0, &hotplug_decision_work, min_sampling_rate);
}

static struct early_suspend auto_hotplug_suspend = {
	.suspend = auto_hotplug_early_suspend,
	.resume = auto_hotplug_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int set_enabled(const char *arg, const struct kernel_param *kp)
{
    int ret = 0;

    ret = param_set_bool(arg, kp);

#if DEBUG
	pr_info("auto_hotplug: enabled is: %d\n", enabled);
#endif

    hotplug_disable(!enabled);

    if(!enabled)
    	schedule_work(&hotplug_online_all_work);

    return ret;
}

static int min_online_cpus_set(const char *arg, const struct kernel_param *kp)
{
    int ret;

    ret = param_set_uint(arg, kp);

    ///minimum rage between 1 - max
    if (min_online_cpus < 1)
    	min_online_cpus = 1;
    else if(min_online_cpus > max_online_cpus)
    	min_online_cpus = min_online_cpus;

#if DEBUG
	pr_info("auto_hotplug: min_online_cpus is: %d\n", min_online_cpus);
#endif

    //online all cores and offline them based on set value
    schedule_work(&hotplug_online_all_work);

    return ret;
}

static int max_online_cpus_set(const char *arg, const struct kernel_param *kp)
{
    int ret;

    ret = param_set_uint(arg, kp);

    ///maximum rage between min - CPUS_AVAILABLE
    if (max_online_cpus < min_online_cpus)
        max_online_cpus = min_online_cpus;
    else if(max_online_cpus > CPUS_AVAILABLE)
    	max_online_cpus = CPUS_AVAILABLE;

#if DEBUG
	pr_info("auto_hotplug: max_online_cpus is: %d\n", max_online_cpus);
#endif

    return ret;
}

static int min_sampling_rate_ms_set(const char *arg, const struct kernel_param *kp)
{
    int ret;

    ret = param_set_uint(arg, kp);

    if (min_sampling_rate_ms <= 0)
    	min_sampling_rate_ms = DEFAULT_SAMPLING_RATE;

    min_sampling_rate = msecs_to_jiffies(min_sampling_rate_ms);

#if DEBUG
	pr_info("auto_hotplug: min_sampling_rate_ms is: %d\n", min_sampling_rate_ms);
#endif

    return ret;
}

static int screen_off_min_freq_set(const char *arg, const struct kernel_param *kp)
{
    int ret;
	int i = 0;
    struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(0);

    ret = param_set_uint(arg, kp);

    for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++)
    {
		if (table[i].frequency >= screen_off_min_freq)
		{
			screen_off_min_freq = table[i].frequency;
			break;
		}
    }

    return ret;
}

static int screen_off_max_freq_set(const char *arg, const struct kernel_param *kp)
{
    int ret;
	int i = 0;
    struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(0);

    ret = param_set_uint(arg, kp);

    for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++)
    {
		if (table[i].frequency >= screen_off_max_freq)
		{
			screen_off_max_freq = table[i].frequency;
			break;
		}
    }

    return ret;
}

static struct kernel_param_ops enabled_ops =
{
    .set = set_enabled,
    .get = param_get_bool,
};

static struct kernel_param_ops min_online_cpus_ops =
{
    .set = min_online_cpus_set,
    .get = param_get_uint,
};

static struct kernel_param_ops max_online_cpus_ops =
{
    .set = max_online_cpus_set,
    .get = param_get_uint,
};

static struct kernel_param_ops min_sampling_rate_ms_ops =
{
    .set = min_sampling_rate_ms_set,
    .get = param_get_uint,
};

static struct kernel_param_ops sampling_rate_scale_ops =
{
    .set = param_set_uint,
    .get = param_get_uint,
};

static struct kernel_param_ops enable_load_threshold_ops =
{
    .set = param_set_uint,
    .get = param_get_uint,
};

static struct kernel_param_ops disable_load_threshold_ops =
{
    .set = param_set_uint,
    .get = param_get_uint,
};

static struct kernel_param_ops screen_off_min_freq_ops =
{
    .set = screen_off_min_freq_set,
    .get = param_get_uint,
};

static struct kernel_param_ops screen_off_max_freq_ops =
{
    .set = screen_off_max_freq_set,
    .get = param_get_uint,
};

module_param_cb(enabled, &enabled_ops, &enabled, 0644);
MODULE_PARM_DESC(enabled, "control auto_hotplug");
module_param_cb(min_online_cpus, &min_online_cpus_ops, &min_online_cpus, 0644);
module_param_cb(max_online_cpus, &max_online_cpus_ops, &max_online_cpus, 0644);
module_param_cb(min_sampling_rate_ms, &min_sampling_rate_ms_ops, &min_sampling_rate_ms, 0644);
module_param_cb(sampling_rate_scale, &sampling_rate_scale_ops, &sampling_rate_scale, 0644);
module_param_cb(enable_load_threshold, &enable_load_threshold_ops, &enable_load_threshold, 0644);
module_param_cb(disable_load_threshold, &disable_load_threshold_ops, &disable_load_threshold, 0644);
module_param_cb(screen_off_min_freq, &screen_off_min_freq_ops, &screen_off_min_freq, 0644);
module_param_cb(screen_off_max_freq, &screen_off_max_freq_ops, &screen_off_max_freq, 0644);

static inline void hotplug_decision_work_fn(struct work_struct *work)
{
	unsigned int running, disable_load, sampling_rate, enable_load, avg_running = 0;
	unsigned int online_cpus, i, j;
#if DEBUG
	unsigned int k;
#endif

	if(!enabled)
		return;

	online_cpus = num_online_cpus();
	disable_load = disable_load_threshold * online_cpus;
	enable_load = enable_load_threshold * online_cpus;
	/*
	 * Multiply nr_running() by 100 so we don't have to
	 * use fp division to get the average.
	 */
	running = nr_running() * 100;

	history[index] = running;

#if DEBUG
	pr_info("auto_hotplug: online_cpus is: %d\n", online_cpus);
	pr_info("auto_hotplug: enable_load is: %d\n", enable_load);
	pr_info("auto_hotplug: disable_load is: %d\n", disable_load);
	pr_info("auto_hotplug: index is: %d\n", index);
	pr_info("auto_hotplug: running is: %d\n", running);
#endif

	/*
	 * Use a circular buffer to calculate the average load
	 * over the sampling periods.
	 * This will absorb load spikes of short duration where
	 * we don't want additional cores to be onlined because
	 * the cpufreq driver should take care of those load spikes.
	 */
	for (i = 0, j = index; i < SAMPLING_PERIODS; i++, j--) {
		avg_running += history[j];
		if (unlikely(j == 0))
			j = INDEX_MAX_VALUE;
	}

	/*
	 * If we are at the end of the buffer, return to the beginning.
	 */
	if (unlikely(index++ == INDEX_MAX_VALUE))
		index = 0;

#if DEBUG
	pr_info("auto_hotplug: array contents: ");
	for (k = 0; k < SAMPLING_PERIODS; k++) {
		 pr_info("%d: %d\t",k, history[k]);
	}
	pr_info("\n");
	pr_info("auto_hotplug: avg_running before division: %d\n", avg_running);
#endif

	avg_running = AVG_NR(avg_running);

#if DEBUG
	pr_info("auto_hotplug: average_running is: %d\n", avg_running);
#endif

	if (likely(!(flags & HOTPLUG_DISABLED))) {
		if (unlikely((avg_running >= ENABLE_ALL_LOAD_THRESHOLD) && (max_online_cpus > online_cpus))) {
#if DEBUG
			pr_info("auto_hotplug: Onlining all CPUs, avg running: %d\n", avg_running);
#endif
			/*
			 * Flush any delayed offlining work from the workqueue.
			 * No point in having expensive unnecessary hotplug transitions.
			 * We still online after flushing, because load is high enough to
			 * warrant it.
			 * We set the paused flag so the sampling can continue but no more
			 * hotplug events will occur.
			 */
			flags |= HOTPLUG_PAUSED;
			if (delayed_work_pending(&hotplug_offline_work))
				cancel_delayed_work(&hotplug_offline_work);
			schedule_work(&hotplug_online_all_work);
			return;
		} else if (flags & HOTPLUG_PAUSED) {
			schedule_delayed_work_on(0, &hotplug_decision_work, min_sampling_rate);
			return;
		} else if ((avg_running >= enable_load) && (max_online_cpus > online_cpus)) {
#if DEBUG
			pr_info("auto_hotplug: Onlining single CPU, avg running: %d\n", avg_running);
#endif
			if (delayed_work_pending(&hotplug_offline_work))
				cancel_delayed_work(&hotplug_offline_work);
			schedule_work(&hotplug_online_single_work);
			return;
		} else if ((avg_running <= disable_load) && (min_online_cpus < online_cpus)) {
			/* Only queue a cpu_down() if there isn't one already pending */
			if (!(delayed_work_pending(&hotplug_offline_work))) {
#if DEBUG
				pr_info("auto_hotplug: Offlining CPU, avg running: %d\n", avg_running);
#endif
				schedule_delayed_work_on(0, &hotplug_offline_work, HZ);
			}
			/* If boostpulse is active, clear the flags */
			if (flags & BOOSTPULSE_ACTIVE) {
				flags &= ~BOOSTPULSE_ACTIVE;
#if DEBUG
				pr_info("auto_hotplug: Clearing boostpulse flags\n");
#endif
			}
		}
	}

	/*
	 * Reduce the sampling rate dynamically based on online cpus.
	 */
	if(!sampling_rate_scale)
		sampling_rate = min_sampling_rate * (online_cpus * online_cpus);
	else
		sampling_rate = min_sampling_rate * (online_cpus * sampling_rate_scale);

#if DEBUG
	pr_info("auto_hotplug: sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
#endif
	schedule_delayed_work_on(0, &hotplug_decision_work, sampling_rate);

}

static inline void __cpuinit hotplug_online_all_work_fn(struct work_struct *work)
{
	int cpu;
	int max = enabled ? max_online_cpus : CPUS_AVAILABLE;

	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu)) && num_online_cpus() < max) {
			cpu_up(cpu);
#if DEBUG
			pr_info("auto_hotplug: CPU%d is up\n", cpu);
#endif
		}
	}

	if(enabled){
		schedule_delayed_work(&hotplug_unpause_work, HZ);
		schedule_delayed_work_on(0, &hotplug_decision_work, min_sampling_rate);
	}
}

static inline void hotplug_offline_all_work_fn(struct work_struct *work)
{
	int cpu;
	int min = enabled ? min_online_cpus : 1;

	for_each_possible_cpu(cpu) {
		if (likely(cpu_online(cpu) && (cpu)) && num_online_cpus() > min) {
			cpu_down(cpu);
#if DEBUG
			pr_info("auto_hotplug: CPU%d is down\n", cpu);
#endif
		}
	}
}

static inline void __cpuinit hotplug_online_single_work_fn(struct work_struct *work)
{
	int cpu;

	/* All CPUs are online, return */
	if (num_online_cpus() >= max_online_cpus)
		return;

	for_each_possible_cpu(cpu) {
		if (cpu) {
			if (!cpu_online(cpu)) {
				cpu_up(cpu);
#if DEBUG
			pr_info("auto_hotplug: CPU%d is up\n", cpu);
#endif
				break;
			}
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, min_sampling_rate);
}

static inline void hotplug_offline_work_fn(struct work_struct *work)
{
	int cpu;

	/* Min online CPUs, return */
	if (num_online_cpus() <= min_online_cpus)
		return;

	for_each_online_cpu(cpu) {
		if (cpu) {
			cpu_down(cpu);
#if DEBUG
			pr_info("auto_hotplug: CPU%d is down\n", cpu);
#endif
			break;
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, min_sampling_rate);
}

static inline void hotplug_unpause_work_fn(struct work_struct *work)
{
	flags &= ~HOTPLUG_PAUSED;
}

void hotplug_disable(bool flag)
{
	if (flags & HOTPLUG_DISABLED && !flag) {
		flags &= ~HOTPLUG_DISABLED;
		flags &= ~HOTPLUG_PAUSED;
#if DEBUG
		pr_info("auto_hotplug: Clearing disable flag\n");
#endif
		schedule_delayed_work_on(0, &hotplug_decision_work, min_sampling_rate);
	} else if (flag && (!(flags & HOTPLUG_DISABLED))) {
		flags |= HOTPLUG_DISABLED;
#if DEBUG
		pr_info("auto_hotplug: Setting disable flag\n");
#endif
		cancel_delayed_work_sync(&hotplug_offline_work);
		cancel_delayed_work_sync(&hotplug_decision_work);
		cancel_delayed_work_sync(&hotplug_unpause_work);
	}
}

inline void hotplug_boostpulse(void)
{

    unsigned int online_cpus;
    online_cpus = num_online_cpus();

	if (unlikely(flags & (EARLYSUSPEND_ACTIVE
		| HOTPLUG_DISABLED)))
		return;

	if (!(flags & BOOSTPULSE_ACTIVE) && (max_online_cpus > online_cpus)) {
		flags |= BOOSTPULSE_ACTIVE;
		/*
		 * If there are less than 2 CPUs online, then online
		 * an additional CPU, otherwise check for any pending
		 * offlines, cancel them and pause for 2 seconds.
		 * Either way, we don't allow any cpu_down()
		 * whilst the user is interacting with the device.
		 */
		if (likely(online_cpus < 2)) {
			cancel_delayed_work_sync(&hotplug_offline_work);
			flags |= HOTPLUG_PAUSED;
			schedule_work(&hotplug_online_single_work);
			schedule_delayed_work(&hotplug_unpause_work, HZ);
		} else {
#if DEBUG
			pr_info("auto_hotplug: %s: %d CPUs online\n", __func__, num_online_cpus());
#endif
			if (delayed_work_pending(&hotplug_offline_work)) {
#if DEBUG
				pr_info("auto_hotplug: %s: Cancelling hotplug_offline_work\n", __func__);
#endif
				cancel_delayed_work(&hotplug_offline_work);
				flags |= HOTPLUG_PAUSED;
				schedule_delayed_work(&hotplug_unpause_work, HZ);
				schedule_delayed_work_on(0, &hotplug_decision_work, min_sampling_rate);
			}
		}
	}
}

int __init auto_hotplug_init(void)
{
	pr_info("auto_hotplug: v0.220 by _thalamus\n");
	pr_info("auto_hotplug: %d CPUs detected\n", CPUS_AVAILABLE);

	// Initial parameters value
    min_sampling_rate = msecs_to_jiffies(min_sampling_rate_ms);

    // max_online_cpus = num_possible_cpus();

	INIT_DELAYED_WORK(&hotplug_decision_work, hotplug_decision_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_unpause_work, hotplug_unpause_work_fn);
	INIT_WORK(&hotplug_online_all_work, hotplug_online_all_work_fn);
	INIT_WORK(&hotplug_online_single_work, hotplug_online_single_work_fn);
	INIT_WORK(&hotplug_offline_all_work, hotplug_offline_all_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_offline_work, hotplug_offline_work_fn);

	/*
	 * Give the system time to boot before fiddling with hotplugging.
	 */
	flags |= HOTPLUG_PAUSED;
	schedule_delayed_work_on(0, &hotplug_decision_work, HZ * 5);
	schedule_delayed_work(&hotplug_unpause_work, HZ * 10);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&auto_hotplug_suspend);
#endif
	return 0;
}
late_initcall(auto_hotplug_init);
