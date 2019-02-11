#include <linux/err.h>

#include <linux/htc_flashlight.h>

int (*htc_flash_front)(int, int);
int (*htc_torch_front)(int, int);

static LIST_HEAD(fl_head);

static int htc_flash_front_wrapper(int mode1, int mode2)
{
	struct htc_flashlight_dev *fl_dev;

	list_for_each_entry(fl_dev, &fl_head, list) {
		if (1 == fl_dev->id && fl_dev->flash_func) {
			return fl_dev->flash_func(fl_dev, mode1, mode2);
		}
	}

	return -EINVAL;
}

static int htc_torch_front_wrapper(int mode1, int mode2)
{
	struct htc_flashlight_dev *fl_dev;

	list_for_each_entry(fl_dev, &fl_head, list) {
		if (1 == fl_dev->id && fl_dev->torch_func) {
			return fl_dev->torch_func(fl_dev, mode1, mode2);
		}
	}

	return -EINVAL;
}

int register_htc_flashlight(struct htc_flashlight_dev *dev)
{
	struct htc_flashlight_dev *tmp;

	list_for_each_entry(tmp, &fl_head, list) {
		if (dev->id == tmp->id) {
			pr_err("%s: id %d already resigtered\n", __func__, dev->id);
			return -EBUSY;
		}
	}

	list_add(&dev->list, &fl_head);
	pr_info("%s: id %d registered for (%pS, %pS)\n", __func__, dev->id, dev->flash_func, dev->torch_func);

	if (dev->id == 1) {
		htc_flash_front = htc_flash_front_wrapper;
		htc_torch_front = htc_torch_front_wrapper;
	}

	return 0;
}

int unregister_htc_flashlight(struct htc_flashlight_dev *dev)
{
	struct htc_flashlight_dev *tmp;

	list_for_each_entry(tmp, &fl_head, list) {
		if (dev->id == tmp->id) {
			if (dev->id == 1) {
				htc_flash_front = NULL;
				htc_torch_front = NULL;
			}
			list_del(&tmp->list);
			pr_info("%s: id %d\n", __func__, dev->id);
			break;
		}
	}

	return 0;
}
