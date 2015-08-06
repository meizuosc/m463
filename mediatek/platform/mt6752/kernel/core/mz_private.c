#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mach/mz_private.h>

#define HW_TYPE_MASK       0xfff00000
#define HW_TYPE_SHIFT      20
#define INTL_VER_MASK      0x000f0000
#define INTL_VER_SHIFT     16
#define OP_TYPE_MASK       0x00000f00
#define OP_TYPE_SHIFT      8
#define COMPILE_VER_MASK   0x0000f000
#define COMPILE_VER_SHIFT  12
#define PR_VER_MASK        0x00000fff
#define PR_VER_SHIFT       0


#define HW_INFO_HW_ID_SHIFT  16
#define HW_INFO_MD_ID_SHIFT  12
#define HW_INFO_MODEM_SHIFT  8
#define HW_INFO_HWVER_SHIFT  0

#define HW_INFO_HW_ID_MASK  0x00FF0000
#define HW_INFO_MD_ID_MASK  0x0000F000
#define HW_INFO_MODEM_MASK  0x00000F00
#define HW_INFO_HWVER_MASK  0x000000FF

struct tag_lk_info g_lk_info = {0,};
int lk_info_setup(const char * lk_info)
{
	memcpy(&g_lk_info, lk_info, sizeof(struct tag_lk_info));
	return 0;
}

unsigned int mz_get_hw_version(void)
{
	unsigned int hw_ver = 0;
	hw_ver = (g_lk_info.hw_info & HW_INFO_HWVER_MASK) >> HW_INFO_HWVER_SHIFT;
	return hw_ver;
}
EXPORT_SYMBOL(mz_get_hw_version);

unsigned int mz_get_hw_id(void)
{
	unsigned int hw_id = 0;
	hw_id = (g_lk_info.hw_info & HW_INFO_HW_ID_MASK) >> HW_INFO_HW_ID_SHIFT;
	return hw_id;
}
EXPORT_SYMBOL(mz_get_hw_id);

unsigned int mz_get_sw_version(void)
{
	return g_lk_info.sw_version;
}
EXPORT_SYMBOL(mz_get_sw_version);

#if 1
unsigned int mz_has_muic(void)
{
	unsigned int hw_ver = 0;
	unsigned int hw_id  = 0;
	unsigned int muic   = 0;

	hw_ver = mz_get_hw_version();
	hw_id  = mz_get_hw_id();

#ifndef MEIZU_M79
	/* m71 */
	if (hw_ver < 5) {
		muic = 1;
	} else if (hw_ver == 5 && 11 == hw_id) {
		/* v0.5 */
		muic = 1;
	} else if (hw_ver == 5 && 101 == hw_id) {
		/* NPI v0.5 */
		muic = 0;
	} else {
		muic = 1;
	}
#else
	/* m79 */
	muic = 1;
	if (hw_ver >= 10)
		muic = 0;
#endif

	return muic;
}
#else
unsigned int mz_has_muic(void)
{
	return 0;
}
#endif
EXPORT_SYMBOL(mz_has_muic);

/*
#  return value:
#  bit0 for china unicom, bit1 for china mobile
#    1: support china unicom,  2: support china mobile
#    3: support china unicom and china mobile
 */
unsigned int mz_get_operator_type(void)
{
	return (g_lk_info.hw_info & HW_INFO_MODEM_MASK) >> HW_INFO_MODEM_SHIFT;
}
EXPORT_SYMBOL(mz_get_operator_type);

static int lk_version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_lk_info.lk_version);
	return 0;
}
static int lk_mode_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_lk_info.lk_mode);
	return 0;
}

static int hw_version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mz_get_hw_version());
	return 0;
}

static int hw_info_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_lk_info.hw_info);
	return 0;
}

static int md_type_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mz_get_operator_type());
	return 0;
}

static int has_muic_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mz_has_muic());
	return 0;
}

static int sw_version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%X\n", g_lk_info.sw_version);
	return 0;
}

static int sn_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", g_lk_info.sn);
	return 0;
}

static int psn_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", g_lk_info.psn);
	return 0;
}

static int colortype_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", g_lk_info.colortype);
	return 0;
}

static int rtx_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_lk_info.rsv[1]);
	return 0;
}
extern int sec_schip_enabled(void);
static int sec_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sec_schip_enabled());
	return 0;
}
static int lcd_id_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_lk_info.rsv[2]);
	return 0;
}
#define PROC_FOPS_RO(name)	\
	static int name##_proc_open(struct inode *inode, struct file *file)	\
	{									\
		return single_open(file, name##_proc_show, PDE_DATA(inode));	\
	}									\
	static const struct file_operations name##_proc_fops = {		\
		.owner          = THIS_MODULE,					\
		.open           = name##_proc_open,				\
		.read           = seq_read,					\
		.llseek         = seq_lseek,					\
		.release        = single_release,				\
	}

#define PROC_ENTRY(name) {__stringify(name), &name##_proc_fops}
PROC_FOPS_RO(lk_version);
PROC_FOPS_RO(lk_mode);
PROC_FOPS_RO(hw_version);
PROC_FOPS_RO(hw_info);
PROC_FOPS_RO(md_type);
PROC_FOPS_RO(has_muic);
PROC_FOPS_RO(sw_version);
PROC_FOPS_RO(sn);
PROC_FOPS_RO(psn);
PROC_FOPS_RO(colortype);
PROC_FOPS_RO(rtx);
PROC_FOPS_RO(sec);
PROC_FOPS_RO(lcd_id);

struct pentry {
	const char *name;
	const struct file_operations *fops;
};
const struct pentry lk_info_entries[] = {
	PROC_ENTRY(lk_version),
	PROC_ENTRY(lk_mode),
	PROC_ENTRY(hw_version),
	PROC_ENTRY(hw_info),
	PROC_ENTRY(md_type),
	PROC_ENTRY(has_muic),
	PROC_ENTRY(sw_version),
	PROC_ENTRY(sn),
	PROC_ENTRY(psn),
	PROC_ENTRY(colortype),
	PROC_ENTRY(rtx),
	PROC_ENTRY(sec),
	PROC_ENTRY(lcd_id),
};
unsigned int mz_system_root_flag = 0;
EXPORT_SYMBOL_GPL(mz_system_root_flag);

static int __init proc_lk_info_init(void)
{
	struct proc_dir_entry *dir_entry = NULL;
	int i = 0;
	
	mz_system_root_flag = g_lk_info.rsv[0]; //root area enable.

	dir_entry = proc_mkdir("lk_info", NULL);
	if (!dir_entry) {
		pr_err("LK_INFO: Failed to create /proc/ entry\n");
		return -ENOMEM;
	}
	
	for (i = 0; i < ARRAY_SIZE(lk_info_entries); i++) {
		if (! proc_create(lk_info_entries[i].name, S_IRUGO, dir_entry, lk_info_entries[i].fops))
			pr_err("LK_INFO: Failed to create /proc/lk_info entry nodes\n");
	}
    return 0;
}
module_init(proc_lk_info_init);
