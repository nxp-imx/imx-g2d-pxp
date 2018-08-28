/*
 *  Copyright (C) 2013-2016 Freescale Semiconductor, Inc.
 *  Copyright 2017-2018 NXP.
 *  All Rights Reserved.
 *
 *  The following programs are the sole property of Freescale Semiconductor Inc.,
 *  and contain its proprietary and confidential information.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <unistd.h>
#include <linux/pxp_device.h>
#include "g2d.h"

#ifdef BUILD_FOR_ANDROID
#include <cutils/log.h>
#define g2d_printf ALOGI
#else
#define g2d_printf printf
#endif

#define PXP_DEV_NAME "/dev/pxp_device"

static int fd = -1;
static int open_count;
static pthread_mutex_t lock;

#define g2d_config_chan(config)						       \
do {									       \
	int ret;							       \
	ret = ioctl(fd, PXP_IOC_CONFIG_CHAN, config);			       \
	if (ret < 0) {							       \
		g2d_printf("%s: failed to config pxp channel\n", __func__);\
		return -1;						       \
	}								       \
} while(0)

struct g2dContext {
	int handle;          /* allocated dma channel handle from PXP*/
	unsigned int blending;
	unsigned int global_alpha_enable;
	unsigned int current_type;
	unsigned char dither;
	unsigned char blend_dim;
};

static unsigned int g2d_pxp_fmt_map(unsigned int format)
{
	switch(format) {
	case G2D_RGB565:
#ifdef BUILD_FOR_ANDROID
		return PXP_PIX_FMT_RGB565;
#else
		return PXP_PIX_FMT_BGR565;
#endif
	case G2D_BGR565:
		return PXP_PIX_FMT_RGB565;
	case G2D_BGRX8888:
		return PXP_PIX_FMT_XRGB32;
	case G2D_BGRA8888:
		return PXP_PIX_FMT_ARGB32;
	case G2D_XRGB8888:
		return PXP_PIX_FMT_BGRX32;
	case G2D_ARGB8888:
		return PXP_PIX_FMT_BGRA32;
	case G2D_RGBA8888:
#ifdef BUILD_FOR_ANDROID
		return PXP_PIX_FMT_ARGB32;
#else
		return PXP_PIX_FMT_ABGR32;
#endif
	case G2D_RGBX8888:
#ifdef BUILD_FOR_ANDROID
		return PXP_PIX_FMT_XRGB32;
#else
		return PXP_PIX_FMT_XBGR32;
#endif
	/* yuv format */
	case G2D_UYVY:
		return PXP_PIX_FMT_UYVY;
	case G2D_VYUY:
		return PXP_PIX_FMT_VYUY;
	case G2D_YUYV:
		return PXP_PIX_FMT_YUYV;
	case G2D_YVYU:
		return PXP_PIX_FMT_YVYU;
	case G2D_I420:
		return PXP_PIX_FMT_YUV420P;
	case G2D_YV12:
		return PXP_PIX_FMT_YVU420P;
	case G2D_NV12:
		return PXP_PIX_FMT_NV12;
	case G2D_NV21:
		return PXP_PIX_FMT_NV21;
	case G2D_NV16:
		return PXP_PIX_FMT_NV16;
	case G2D_NV61:
		return PXP_PIX_FMT_NV61;
	default:
		g2d_printf("%s: unsupported format 0x%x\n",
			   __func__, format);
		break;
	}

	return 0;
}

static int g2d_get_bpp(unsigned int format)
{
	switch(format) {
	case G2D_RGB565:
		return 16;
	case G2D_BGRX8888:
	case G2D_BGRA8888:
	case G2D_RGBA8888:
	case G2D_RGBX8888:
	case G2D_ARGB8888:
	case G2D_XRGB8888:
	case G2D_ABGR8888:
	case G2D_XBGR8888:
		return 32;
	case G2D_UYVY:
	case G2D_YUYV:
	case G2D_VYUY:
	case G2D_YVYU:
		return 16;
	/* for the multi-plane format,
	 * only return the bits number
	 * for Y plane
	 */
	case G2D_NV12:
	case G2D_NV21:
	case G2D_NV16:
	case G2D_NV61:
	case G2D_YV12:
	case G2D_I420:
		return 8;
	default:
		g2d_printf("%s: unsupported format for getting bpp\n", __func__);
	}
	return 0;
}

int g2d_open(void **handle)
{
	int ret;
	static int channel;
	struct g2dContext *context;

	if (handle == NULL) {
		g2d_printf("%s: invalid handle\n", __func__);
		return -1;
	}

	context = (struct g2dContext *)calloc(1, sizeof(struct g2dContext));
	if (context == NULL) {
		g2d_printf("malloc memory failed for g2dcontext!\n");
		goto err2;
	}

	pthread_mutex_lock(&lock);
	if (++open_count == 1 || fd < 0) {
		fd = open(PXP_DEV_NAME, O_RDWR);
		if (fd < 0) {
			g2d_printf("open pxp device failed!\n");
			goto err1;
		}
		ret = ioctl(fd, PXP_IOC_GET_CHAN, &channel);
		if (ret < 0) {
			g2d_printf("%s: failed to get pxp channel\n",
				    __func__);
			goto err0;
		}
	}
	context->handle = channel;
	pthread_mutex_unlock(&lock);

	*handle = (void*)context;
	return 0;
err0:
	close(fd);
	open_count--;
err1:
	pthread_mutex_unlock(&lock);
	free(context);
err2:
	*handle = NULL;
	return -1;
}

int g2d_close(void *handle)
{
	int ret;
	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL) {
		g2d_printf("%s: invalid handle\n", __func__);
		return -1;
	}

	pthread_mutex_lock(&lock);
	if (!open_count) {
		pthread_mutex_unlock(&lock);
		return 0;
	}

	if (open_count == 1 && fd > 0) {
		ret = ioctl(fd, PXP_IOC_PUT_CHAN, &context->handle);
		if (ret < 0) {
			pthread_mutex_unlock(&lock);
			g2d_printf("%s: failed to put pxp channel!\n",
				   __func__);
			return -1;
		}
		close(fd);
		fd = -1;
	}
	open_count--;
	pthread_mutex_unlock(&lock);

	free(context);
	handle = NULL;

	return 0;
}

int g2d_make_current(void *handle, enum g2d_hardware_type type)
{
	struct g2dContext *context = (struct g2dContext*)handle;

	if (context == NULL) {
		g2d_printf("%s: invalid handle\n", __func__);
		return -1;
	}

	if (context->current_type == type)
		return 0;

	switch(type) {
	case G2D_HARDWARE_2D:
		context->current_type = type;
		break;
	default:
		g2d_printf("%s: unsupported hardware type %d\n", __func__, type);
		return -1;
	}
	return 0;
}

int g2d_query_feature(void *handle, enum g2d_feature feature, int *available)
{
    struct g2dContext *context = (struct g2dContext*)handle;

    if (context == NULL) {
        g2d_printf("%s: invalid handle\n", __func__);
        return -1;
    }

    if(!available) {
        g2d_printf("%s: invalid param\n", __func__);
        return -1;
    }

    switch(feature) {
        case G2D_SCALING:
        case G2D_SRC_YUV:
        case G2D_DST_YUV:
            *available = 1;
            break;
        default:
            *available = 0;
            break;
    }

    return 0;
}

int g2d_query_cap(void *handle, enum g2d_cap_mode cap, int *enable)
{
	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL) {
		g2d_printf("%s: invalid handle\n", __func__);
		return -1;
	}

	if (enable == NULL)
		return -1;

	switch(cap) {
	case G2D_BLEND:
		*enable = (context->blending == 1);
		break;
	case G2D_GLOBAL_ALPHA:
		*enable = (context->global_alpha_enable == 1);
		break;
	default:
		g2d_printf("%s: unsupported capability %d\n", __func__, cap);
		return -1;
	}

	return 0;
}

int g2d_cache_op(struct g2d_buf *buf, enum g2d_cache_mode op)
{
	int ret;
	unsigned int Bytes;
	void *Logical;
	struct pxp_mem_flush flush;

	if (!buf) {
		g2d_printf("%s: invalid buffer !\n", __func__);
		return -1;
	}

	Bytes = buf->buf_size;
	Logical = buf->buf_vaddr;

	if (!Bytes || !Logical) {
		g2d_printf("%s: invalid buffer data !\n", __func__);
		return -1;
	}

	switch (op) {
	case G2D_CACHE_CLEAN:
		flush.type = CACHE_CLEAN;
		break;
	case G2D_CACHE_FLUSH:
		flush.type = CACHE_FLUSH;
		break;
	case G2D_CACHE_INVALIDATE:
		flush.type = CACHE_INVALIDATE;
		break;
	default:
		g2d_printf("%s: invalid cache op !\n", __func__);
		return -1;
	}

	flush.handle = *(unsigned int *)buf->buf_handle;
	ret = ioctl(fd, PXP_IOC_FLUSH_PHYMEM, &flush);
	if (ret < 0) {
		g2d_printf("%s: flush dma buffer failed\n", __func__);
		return -1;
	}

	return 0;
}

int g2d_enable(void *handle, enum g2d_cap_mode cap)
{
	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL) {
		g2d_printf("%s: invalid handle\n", __func__);
		return -1;
	}

	switch(cap) {
	case G2D_BLEND:
		context->blending = 1;
		break;
	case G2D_GLOBAL_ALPHA:
		context->global_alpha_enable = 1;
		break;
	/*TODO PXP doesn't support dithering yet */
	default:
		g2d_printf("%s: unknown cap %d request\n", __func__, cap);
		return -1;
	}

	return 0;
}

int g2d_disable(void *handle, enum g2d_cap_mode cap)
{
	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL) {
		g2d_printf("%s: invalid handle\n", __func__);
		return -1;
	}

	switch(cap) {
	case G2D_BLEND:
		context->blending = 0;
		break;
	case G2D_GLOBAL_ALPHA:
		context->global_alpha_enable = 0;
		break;
	default:
		g2d_printf("%s: unknown cap %d request\n", __func__, cap);
		return -1;
	}

	return 0;
}

struct g2d_buf *g2d_alloc(int size, int cacheable)
{
	int ret;
	void *addr;
	struct g2d_buf *buf = NULL;
	struct pxp_mem_desc mem_desc;

	buf = (struct g2d_buf*)calloc(1, sizeof(struct g2d_buf));
	if (buf ==  NULL) {
		g2d_printf("%s: malloc g2d_buf failed\n", __func__);
		return NULL;
	}

	buf->buf_handle = calloc(1, sizeof(unsigned int));
	if (buf->buf_handle == NULL)
		goto err;

	memset(&mem_desc, 0, sizeof(mem_desc));
	mem_desc.size  = size;
	mem_desc.mtype = cacheable ? MEMORY_TYPE_CACHED : MEMORY_TYPE_UNCACHED;

	ret = ioctl(fd, PXP_IOC_GET_PHYMEM, &mem_desc);

	if (ret < 0) {
		g2d_printf("%s: get pxp physical memory failed, ret = %d\n",
			   __func__, ret);
		goto err0;
	}

	addr = mmap(0, mem_desc.size, PROT_READ | PROT_WRITE,
		    MAP_SHARED, fd, mem_desc.phys_addr);
	if (addr < 0) {
		g2d_printf("%s: map buffer failed\n", __func__);
		ioctl(fd, PXP_IOC_PUT_PHYMEM, &mem_desc);
		goto err0;
	}
	mem_desc.virt_uaddr = (unsigned int)addr;
	*(unsigned int *)buf->buf_handle = mem_desc.handle;
	buf->buf_vaddr = addr;
	buf->buf_paddr = (int)mem_desc.phys_addr;
	buf->buf_size  = mem_desc.size;

	return buf;
err0:
	free(buf->buf_handle);
err:
	free(buf);
	return NULL;
}

void g2d_fill_param(struct pxp_layer_param *param,
		    struct g2d_surface *surf)
{
	param->width  = surf->width;
	param->height = surf->height;
	param->stride = surf->stride * g2d_get_bpp(surf->format) >> 3;
	param->paddr  = surf->planes[0];
	param->pixel_fmt = g2d_pxp_fmt_map(surf->format);
}

static void g2d_fill_rect(struct g2d_surface *surf,
			  struct rect *rect)
{
	rect->top    = surf->top;
	rect->left   = surf->left;
	rect->width  = surf->right  - surf->left;
	rect->height = surf->bottom - surf->top;
}

int g2d_free(struct g2d_buf *buf)
{
	int ret;
	struct pxp_mem_desc mem_desc;

	if (buf == NULL) {
		g2d_printf("%s: Invalid g2d_buf to be freed\n", __func__);
		return -1;
	}

	munmap(buf->buf_vaddr, buf->buf_size);

	memset(&mem_desc, 0, sizeof(struct pxp_mem_desc));
	mem_desc.handle = *(unsigned int *)buf->buf_handle;
	ret = ioctl(fd, PXP_IOC_PUT_PHYMEM, &mem_desc);

	if (ret < 0) {
		g2d_printf("%s: free pxp physical memory failed\n", __func__);
		return -1;
	}

	free(buf->buf_handle);
	free(buf);
	return 0;
}

#define PXP_COPY_THRESHOLD (16*16*4)
int g2d_copy(void *handle, struct g2d_buf *d, struct g2d_buf* s, int size)
{
	unsigned int blit_size;
	struct pxp_config_data pxp_conf;
	struct pxp_layer_param *src_param = NULL, *out_param = NULL, *ol_param;
	struct pxp_alpha *s0_alpha, *s1_alpha;

	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL || s == NULL || d == NULL) {
		g2d_printf("%s: null pointer access\n", __func__);
		return -1;
	}

	memset(&pxp_conf, 0, sizeof(struct pxp_config_data));

	src_param = &(pxp_conf.s0_param);
	ol_param = &(pxp_conf.ol_param[0]);
	out_param = &(pxp_conf.out_param);

	s0_alpha = &src_param->alpha;
	s1_alpha = &ol_param->alpha;

	if (size < PXP_COPY_THRESHOLD) {
		memcpy(d->buf_vaddr, s->buf_vaddr, size);
		return 0;
	}
	else if (size <= PXP_COPY_THRESHOLD * 4096) {
		src_param->width = PXP_COPY_THRESHOLD >> 2;
	}
	else {
		src_param->width = PXP_COPY_THRESHOLD;
	}

	src_param->stride = src_param->width;
	src_param->pixel_fmt = PXP_PIX_FMT_ARGB32;
	src_param->height = size / (src_param->width << 2);
	if (src_param->height > 16384)
		src_param->height = 16384;

	memcpy(out_param, src_param, sizeof(struct pxp_layer_param));
	out_param->pixel_fmt = PXP_PIX_FMT_ARGB32;
	src_param->paddr = s->buf_paddr;
	out_param->paddr = d->buf_paddr;

	memcpy(ol_param, out_param, sizeof(struct pxp_layer_param));
	ol_param->paddr = d->buf_paddr;
	ol_param->pixel_fmt = PXP_PIX_FMT_ARGB32;

	blit_size = src_param->width * src_param->height * 4;
	pxp_conf.handle = context->handle;
	pxp_conf.proc_data.drect.top = 0;
	pxp_conf.proc_data.drect.left = 0;
	pxp_conf.proc_data.drect.width = src_param->width;
	pxp_conf.proc_data.drect.height = src_param->height;
	memcpy(&pxp_conf.proc_data.srect, &pxp_conf.proc_data.drect,
				sizeof(pxp_conf.proc_data.drect));
	pxp_conf.proc_data.combine_enable = 1;

	/* SRC */
	s1_alpha->alpha_mode = ALPHA_MODE_STRAIGHT;
	s1_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
	s1_alpha->color_mode = COLOR_MODE_STRAIGHT;
	s0_alpha->factor_mode = FACTOR_MODE_ONE;

	/* DST */
	s0_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
	s0_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
	s0_alpha->color_mode = COLOR_MODE_STRAIGHT;
	s1_alpha->factor_mode = FACTOR_MODE_ZERO;

	g2d_config_chan(&pxp_conf);

	if (blit_size == size)
		return 0;
	else if (size - blit_size > src_param->width * 4) {
		struct g2d_buf subs, subd;
		subs.buf_size = d->buf_size - blit_size;
		subd.buf_paddr = d->buf_paddr + blit_size;
		subd.buf_vaddr = (void*)(((int)d->buf_vaddr) + blit_size);

		subs.buf_size = s->buf_size - blit_size;;
		subs.buf_paddr = s->buf_paddr + blit_size;
		subs.buf_vaddr = (void*)(((int)s->buf_vaddr) + blit_size);
		return g2d_copy(handle, &subs, &subd, size - blit_size);
	}
	else {
		size = size - blit_size;
		memcpy(d->buf_vaddr + blit_size, s->buf_vaddr + blit_size, size);
		return 0;
	}
}

int g2d_clear(void *handle, struct g2d_surface *area)
{
	struct pxp_config_data pxp_conf;
	struct pxp_layer_param *out_param;
	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL) {
		g2d_printf("%s: invalid handle\n", __func__);
		return -1;
	}

	if (area == NULL) {
		g2d_printf("%s: invalid clear area\n", __func__);
		return -1;
	}

	memset(&pxp_conf, 0, sizeof(struct pxp_config_data));
	out_param = &(pxp_conf.out_param);
	out_param->pixel_fmt = g2d_pxp_fmt_map(area->format);
	if (!out_param->pixel_fmt) {
		g2d_printf("%s: unsupported output format\n", __func__);
		return -1;
	}

	out_param->width  = area->stride;
	out_param->height = area->height;
	out_param->stride = area->stride * g2d_get_bpp(area->format) >> 3;
	out_param->paddr  = area->planes[0];

	pxp_conf.proc_data.fill_en = 1;
	pxp_conf.proc_data.bgcolor = area->clrcolor;
	pxp_conf.proc_data.drect.width = area->width;
	pxp_conf.proc_data.drect.height = area->height;

	pxp_conf.handle = context->handle;
	g2d_config_chan(&pxp_conf);

	return 0;
}

int g2d_blit(void *handle, struct g2d_surface *src, struct g2d_surface *dst)
{
	struct pxp_config_data pxp_conf;
	struct pxp_proc_data *proc_data;
	struct pxp_alpha *s0_alpha, *s1_alpha;
	struct pxp_layer_param *src_param, *out_param, *third_param = NULL;
	unsigned int srcWidth,srcHeight,dstWidth,dstHeight;
	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL) {
		g2d_printf("%s: Invalid handle!\n", __func__);
		return -1;
	}

	if(!src || !dst)
	{
		g2d_printf("%s: Invalid src and dst parameters!\n", __func__);
		return -1;
	}

	if (!context->blend_dim) {
		srcWidth = src->right - src->left;
		srcHeight = src->bottom - src->top;

		if(srcWidth <=0 || srcHeight <= 0 || srcWidth > src->width || srcHeight > src->height || src->width > src->stride)
		{
			g2d_printf("%s: Invalid src rect, left %d, top %d, right %d, bottom %d, width %d, height %d, stride %d!\n",
					__FUNCTION__, src->left, src->top, src->right, src->bottom, src->width, src->height, src->stride);
			return -1;
		}

		if(!src->planes[0])
		{
			g2d_printf("%s: Invalid src planes[0] pointer=0x%x !\n", __FUNCTION__, src->planes[0]);
			return -1;
		}
	} else {
		g2d_printf("%s: dim blending is not supported yet!\n", __func__);
		return -1;
	}

	dstWidth = dst->right - dst->left;
	dstHeight = dst->bottom - dst->top;

	if(dstWidth <=0 || dstHeight <= 0 || dstWidth > dst->width || dstHeight > dst->height || dst->width > dst->stride)
	{
		g2d_printf("%s: Invalid dst rect, left %d, top %d, right %d, bottom %d, width %d, height %d, stride %d!\n",
				__FUNCTION__, dst->left, dst->top, dst->right, dst->bottom, dst->width, dst->height, dst->stride);
		return -1;
	}

	if(!dst->planes[0])
	{
		g2d_printf("%s: Invalid dst planes[0] pointer=0x%x !\n", __FUNCTION__, dst->planes[0]);
		return -1;
	}

	if (src->format >= G2D_NV12 && src->global_alpha == 0xff) {
		context->blending = 0;
	}
	memset(&pxp_conf, 0, sizeof(struct pxp_config_data));
	proc_data = &pxp_conf.proc_data;

	src_param = &(pxp_conf.s0_param);
	g2d_fill_param(src_param, src);
	src_param->width = src->stride;
	src_param->height = src->height;

	out_param = &(pxp_conf.out_param);
	g2d_fill_param(out_param, dst);

	if (context->blending) {
		third_param = &(pxp_conf.ol_param[0]);
		g2d_fill_param(third_param, dst);
	}
	switch (dst->rot) {
	case G2D_ROTATION_0:
		proc_data->rotate = 0;
		break;
	case G2D_ROTATION_90:
		proc_data->rotate = 90;
		break;
	case G2D_ROTATION_180:
		proc_data->rotate = 180;
		break;
	case G2D_ROTATION_270:
		proc_data->rotate = 270;
		break;
	case G2D_FLIP_H:
		proc_data->hflip  = 1;
		break;
	case G2D_FLIP_V:
		proc_data->vflip  = 1;
		break;
	default:
		break;
	}

	g2d_fill_rect(dst, &proc_data->drect);

	/* need do alpha blending */
	if (context->blending) {
		proc_data->combine_enable = 1;
		proc_data->alpha_mode = ALPHA_MODE_PORTER_DUFF;
		s0_alpha = &src_param->alpha;
		s1_alpha = &third_param->alpha;

		switch (src->blendfunc & 0xf) {
		case G2D_ZERO:	/* Fs = 0 */
			s1_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s1_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s1_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s0_alpha->factor_mode = FACTOR_MODE_ZERO;
			break;
		case G2D_ONE:	/* Fs = 1 */
			s1_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s1_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s1_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s0_alpha->factor_mode = FACTOR_MODE_ONE;
			break;
		case G2D_DST_ALPHA:	/* Fs = Ad'' */
			s1_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s1_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s1_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s0_alpha->factor_mode = FACTOR_MODE_STRAIGHT;
			break;
		case G2D_ONE_MINUS_DST_ALPHA:	/* Fs = 1 - Ad'' */
			s1_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s1_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s1_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s0_alpha->factor_mode = FACTOR_MODE_INVERSED;
			break;
		default:
			printf("%s: unsupported alpha mode for source\n", __func__);
			break;
		}
		if (context->global_alpha_enable) {
			s1_alpha->global_alpha_mode  = GLOBAL_ALPHA_MODE_SCALE;
			s1_alpha->global_alpha_value = dst->global_alpha;
		}

		switch(dst->blendfunc & 0xf) {
		case G2D_ZERO:		/* Fd = 0 */
			s0_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s0_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s0_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s1_alpha->factor_mode = FACTOR_MODE_ZERO;
			break;
		case G2D_ONE:		/* Fd = 1 */
			s0_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s0_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s0_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s1_alpha->factor_mode = FACTOR_MODE_ONE;
			break;
		case G2D_SRC_ALPHA:	/* Fd = As'' */
			s0_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s0_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s0_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s1_alpha->factor_mode = FACTOR_MODE_STRAIGHT;
			break;
		case G2D_ONE_MINUS_SRC_ALPHA:	/* Fd = 1 - As'' */
			s0_alpha->alpha_mode  = ALPHA_MODE_STRAIGHT;
			s0_alpha->global_alpha_mode = GLOBAL_ALPHA_MODE_OFF;
			s0_alpha->color_mode  = COLOR_MODE_STRAIGHT;
			s1_alpha->factor_mode = FACTOR_MODE_INVERSED;
			break;
		default:
			printf("%s: unsupported alpha mode destination\n", __func__);
			break;
		}
		if (context->global_alpha_enable) {
			s0_alpha->global_alpha_mode  = GLOBAL_ALPHA_MODE_SCALE;
			s0_alpha->global_alpha_value = src->global_alpha;
		}

		if (src->blendfunc & G2D_PRE_MULTIPLIED_ALPHA)
			s0_alpha->color_mode = COLOR_MODE_MULTIPLY;
		if (dst->blendfunc & G2D_PRE_MULTIPLIED_ALPHA)
			s1_alpha->color_mode = COLOR_MODE_MULTIPLY;
	}

	g2d_fill_rect(src, &proc_data->srect);

	pxp_conf.handle = context->handle;
	g2d_config_chan(&pxp_conf);

	return 0;
}

int g2d_flush(void *handle)
{
	int ret;
	struct g2dContext *context = (struct g2dContext *)handle;

	if (context == NULL) {
		g2d_printf("%s: Invalid handle!\n", __func__);
		return -1;
	}

	ret = ioctl(fd, PXP_IOC_START_CHAN, &context->handle);
	if (ret < 0) {
		g2d_printf("%s: failed to commit pxp task\n", __func__);
		return -1;
	}

	return 0;
}

int g2d_finish(void *handle)
{
	int ret;
	struct g2dContext *context = (struct g2dContext *)handle;
	struct pxp_chan_handle chan_handle;

	if (context == NULL) {
		g2d_printf("%s: Invalid handle!\n", __func__);
		return -1;
	}

	ret = ioctl(fd, PXP_IOC_START_CHAN, &context->handle);
	if (ret < 0) {
		g2d_printf("%s: failed to commit pxp task\n", __func__);
		return -1;
	}

	chan_handle.handle = context->handle;
	ret = ioctl(fd, PXP_IOC_WAIT4CMPLT, &chan_handle);
	if (ret < 0) {
		g2d_printf("%s: failed to wait task complete\n", __func__);
		return -1;
	}

	return 0;
}
