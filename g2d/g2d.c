/*
 *  Copyright (C) 2013-2016 Freescale Semiconductor, Inc.
 *  Copyright 2017-2022 NXP.
 *  NXP Confidential.
 *
 *  This software is owned or controlled by NXP and may only be used strictly
 *  in accordance with the applicable license terms. By expressly accepting
 *  such terms or by downloading, installing, activating and/or otherwise using
 *  the software, you are agreeing that you have read, and that you agree to
 *  comply with and are bound by, such license terms. If you do not agree to be
 *  bound by the applicable license terms, then you may not retain, install,
 *  activate or otherwise use the software.
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
#include <linux/const.h>
#include <linux/pxp_device.h>
#include <math.h>
#include <linux/dma-buf.h>
#ifdef BUILD_FOR_ANDROID
#include <linux/dma-buf-imx.h>
#endif
#include <linux/dma-heap.h>
#include <errno.h>
#include "g2d.h"
#include "g2dExt.h"

#ifdef BUILD_FOR_ANDROID
#include <cutils/log.h>
#define g2d_printf ALOGI
#else
#define g2d_printf printf
#endif

#define PXP_DEV_NAME "/dev/pxp_device"

#define DEVPATH "/dev/dma_heap"
#define HEAPNAME_CACHED "linux,cma"
#define HEAPNAME_UNCACHED "linux,cma-uncached"

/* to align the pointer to the (next) page boundary */
#define __PAGE_ALIGN(addr) __ALIGN(addr, __PAGE_SIZE)

#define __PAGE_SHIFT 12
#define __PAGE_SIZE (_AC(1, UL) << __PAGE_SHIFT)

#define __ALIGN(x, a) __ALIGN_MASK(x, (typeof(x))(a)-1)
#define __ALIGN_MASK(x, mask) (((x) + (mask)) & ~(mask))

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

#define g2dFALSE 0
#define g2dTRUE  1

typedef int g2dBOOL;
typedef signed int g2dINT32;

#ifdef __cplusplus
	#define g2dNULL 0
#else
	#define g2dNULL ((void *) 0)
#endif

typedef struct _g2dRECT
{
	g2dINT32 left;
	g2dINT32 top;
	g2dINT32 right;
	g2dINT32 bottom;
}g2dRECT;

struct g2dContext {
	int handle;          /* allocated dma channel handle from PXP*/
	unsigned int blending;
	unsigned int global_alpha_enable;
	unsigned int current_type;
	unsigned char dither;
	unsigned char blend_dim;

	/* Clipping 2D flag */
	g2dBOOL	clipping2D;

	/* Clipping rectangle */
	g2dRECT clipRect2D;
};

struct dma_buf {
    unsigned long       paddr;
    void                *vaddr;
    int                 size;
    void                *handle;
    int                 dmafd;
};

static int devfd = -1;
static int g2d_dev_open(int cacheable);
static int g2d_dev_close(int devfd);

static int checkSurfaceRect(struct g2d_surface *surface)
{
	int rectWidth, rectHeight;

	if(!surface)
	{
		g2d_printf("%s: Invalid g2d_surface parameters!\n", __FUNCTION__);
		return G2D_STATUS_FAIL;
	}

	rectWidth = surface->right - surface->left;
	rectHeight = surface->bottom - surface->top;

	if(rectWidth <=0 || rectHeight <= 0 || rectWidth > surface->width || rectHeight > surface->height || surface->width > surface->stride)
	{
		g2d_printf("%s: Invalid src rect, left %d, top %d, right %d, bottom %d, width %d, height %d, stride %d!\n",
				__FUNCTION__, surface->left, surface->top, surface->right, surface->bottom, surface->width, surface->height, surface->stride);
		return G2D_STATUS_FAIL;
	}

	return G2D_STATUS_OK;
}

static void updateSurfaceRect(struct g2d_surface *src, struct g2d_surface *dst, g2dRECT clipRect2D)
{
	int sr;        /* Source rectangle resize in pixels due to clipping */
	double sx, sy; /* Scaling factor in x and y */

	switch(src->rot)
	{
	case G2D_ROTATION_0:
	default:
		sx = (double)(src->right - src->left) / (double)(dst->right - dst->left);
		sy = (double)(src->bottom - src->top) / (double)(dst->bottom - dst->top);

		sr = floor((double)(clipRect2D.left - dst->left) * sx);
		if (sr > 0) {
			src->left = src->left + sr;
		}

		sr = floor((double)(clipRect2D.top - dst->top) * sy);
		if (sr > 0) {
			src->top = src->top + sr;
		}

		sr = floor((double)(dst->right - clipRect2D.right) * sx);
		if (sr > 0) {
			src->right = src->right - sr;
		}

		sr = floor((double)(dst->bottom - clipRect2D.bottom) * sy);
		if (sr > 0) {
			src->bottom = src->bottom - sr;
		}

		break;
	case G2D_ROTATION_90:
	case G2D_ROTATION_270:
		sx = (double)(src->right - src->left) / (double)(dst->bottom - dst->top);
		sy = (double)(src->bottom - src->top) / (double)(dst->right - dst->left);

		sr = floor((double)(clipRect2D.top - dst->top) * sx);
		if (sr > 0)
		{
			src->left = src->left + sr;
		}

		sr = floor((double)(clipRect2D.left - dst->left) * sy);
		if (sr > 0)
		{
			src->top = src->top + sr;
		}

		sr = floor((double)(dst->right - clipRect2D.right) * sx);
		if (sr > 0)
		{
			src->bottom = src->bottom - sr;
		}

		sr = floor((double)(dst->bottom - clipRect2D.bottom) * sy);
		if (sr > 0)
		{
			src->right = src->right	- sr;
		}

	break;
	}

	if(src->width != 0 && src->height != 0)
	{
		src->right = (src->right < src->width) ? src->right : src->width;
		src->bottom = (src->bottom < src->height) ? src->bottom : src->height;
	}

	if (dst->left < clipRect2D.left)
	{
		dst->left = clipRect2D.left;
	}
	if (dst->top < clipRect2D.top)
	{
		dst->top = clipRect2D.top;
	}
	if (dst->right > clipRect2D.right)
	{
		dst->right = clipRect2D.right;
	}
	if (dst->bottom > clipRect2D.bottom)
	{
		dst->bottom = clipRect2D.bottom;
	}
}

static unsigned int g2d_pxp_fmt_map(unsigned int format)
{
	switch(format) {
	case G2D_RGB565:
		return PXP_PIX_FMT_RGB565;
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
	devfd = g2d_dev_open(0);

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
	g2d_dev_close(devfd);

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
        case G2D_ROTATION:
            *available = 1;
            break;
        case G2D_WARP_DEWARP:
            *available = 0;
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
	case G2D_WARPING:
		return G2D_STATUS_NOT_SUPPORTED;
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
	case G2D_WARPING:
		return G2D_STATUS_NOT_SUPPORTED;
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

int g2d_set_clipping(void *handle, int left, int top, int right, int bottom)
{
	struct g2dContext * context = g2dNULL;

	context = (struct g2dContext *)handle;
	if(!context)
	{
		g2d_printf("%s: Invalid handle !\n", __FUNCTION__);
		return G2D_STATUS_FAIL;
	}

	context->clipRect2D.left = left;
	context->clipRect2D.top = top;
	context->clipRect2D.right = right;
	context->clipRect2D.bottom = bottom;

	context->clipping2D = g2dTRUE;

	return G2D_STATUS_OK;
}

void g2d_fill_param(struct pxp_layer_param *param,
		    struct g2d_surface *surf)
{
	param->width  = surf->width;
	param->height = surf->height;
	param->stride = surf->stride * g2d_get_bpp(surf->format) >> 3;
	param->paddr  = surf->planes[0];
	param->paddr_u  = surf->planes[1];
	param->paddr_v  = surf->planes[2];
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
		subd.buf_vaddr = d->buf_vaddr + blit_size;

		subs.buf_size = s->buf_size - blit_size;;
		subs.buf_paddr = s->buf_paddr + blit_size;
		subs.buf_vaddr = s->buf_vaddr + blit_size;
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

	//BGR to RGB
	area->clrcolor = ((area->clrcolor >> 16) & 0xff) |
                 ((area->clrcolor << 16) & 0xff0000) |
                 ((area->clrcolor & 0xff00ff00));

	out_param->width  = area->right - area->left;
	out_param->height = area->bottom - area->top;
	out_param->stride = area->stride * g2d_get_bpp(area->format) >> 3;
	out_param->paddr  = area->planes[0] + (area->top * area->stride + area->left)*(g2d_get_bpp(area->format) >> 3);

	pxp_conf.proc_data.fill_en = 1;
	pxp_conf.proc_data.bgcolor = area->clrcolor;
	pxp_conf.proc_data.drect.width = area->right - area->left;
	pxp_conf.proc_data.drect.height = area->bottom - area->top;

	pxp_conf.handle = context->handle;
	g2d_config_chan(&pxp_conf);

	return 0;
}

int g2d_blit_wrap(void *handle, struct g2d_surface *src, struct g2d_surface *dst)
{
	struct pxp_config_data pxp_conf;
	struct pxp_proc_data *proc_data;
	struct pxp_alpha *s0_alpha, *s1_alpha;
	struct pxp_layer_param *src_param, *out_param, *third_param = NULL;
	unsigned int srcWidth,srcHeight,dstWidth,dstHeight;
	struct g2dContext *context = (struct g2dContext *)handle;
	g2dRECT srcRect = {0,0,0,0}, dstRect = {0,0,0,0};
	int ret = G2D_STATUS_FAIL;

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

	if(context->clipping2D && (src->rot == G2D_ROTATION_0))
	{
		srcRect.left = src->left; srcRect.top = src->top;
		srcRect.right = src->right; srcRect.bottom = src->bottom;
		dstRect.left = dst->left; dstRect.top = dst->top;
		dstRect.right = dst->right; dstRect.bottom = dst->bottom;
		updateSurfaceRect(src, dst, context->clipRect2D);

		/* early exit if no dirty region in clipping area */
		if (src->left >= src->right || src->top >= src->bottom)
		{
			ret = G2D_STATUS_OK;
			goto done;
		}

		/* early exit if no dirty region in clipping area */
		if (dst->left >= dst->right || dst->top >= dst->bottom)
		{
			ret = G2D_STATUS_OK;
			goto done;
		}

		ret = checkSurfaceRect(src);
		if(ret != G2D_STATUS_OK)
		{
			g2d_printf("%s: Invalid src clipping surface !\n", __FUNCTION__);
			ret = G2D_STATUS_FAIL;
			goto done;
		}

		ret = checkSurfaceRect(dst);
		if(ret != G2D_STATUS_OK)
		{
			g2d_printf("%s: Invalid dst clipping surface !\n", __FUNCTION__);
			ret = G2D_STATUS_FAIL;
			goto done;
		}
	}

	memset(&pxp_conf, 0, sizeof(struct pxp_config_data));
	proc_data = &pxp_conf.proc_data;

	src_param = &(pxp_conf.s0_param);
	g2d_fill_param(src_param, src);
	src_param->width = src->stride;
	src_param->height = src->height;

	out_param = &(pxp_conf.out_param);
	g2d_fill_param(out_param, dst);

	out_param->width = dst->right - dst->left;
	out_param->height = dst->bottom - dst->top;
	out_param->paddr = dst->planes[0] + (dst->top * dst->stride + dst->left)*(g2d_get_bpp(dst->format) >> 3);

	if (context->blending && (src->rot == G2D_ROTATION_0)) {
		third_param = &(pxp_conf.ol_param[0]);
		g2d_fill_param(third_param, dst);
		third_param->width = dst->right - dst->left;
		third_param->height = dst->bottom - dst->top;
		third_param->paddr = dst->planes[0] + (dst->top * dst->stride + dst->left)*(g2d_get_bpp(dst->format) >> 3);
	}
	switch (src->rot) {
	case G2D_ROTATION_0:
		proc_data->rotate = 0;
		break;
	case G2D_ROTATION_90:
		proc_data->rotate = 270;
		break;
	case G2D_ROTATION_180:
		proc_data->rotate = 180;
		break;
	case G2D_ROTATION_270:
		proc_data->rotate = 90;
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
	if (context->blending && (src->rot == G2D_ROTATION_0)) {
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

	pxp_conf.proc_data.drect.left = 0;
	pxp_conf.proc_data.drect.top = 0;

	pxp_conf.handle = context->handle;
	g2d_config_chan(&pxp_conf);

	return 0;

done:
	if(context->clipping2D)
	{
		src->left = srcRect.left; src->top = srcRect.top;
		src->right = srcRect.right; src->bottom = srcRect.bottom;

		dst->left = dstRect.left; dst->top = dstRect.top;
		dst->right = dstRect.right; dst->bottom = dstRect.bottom;
		context->clipping2D = g2dFALSE;
	}

	return ret;
}

int g2d_blit(void *handle, struct g2d_surface *src, struct g2d_surface *dst)
{
	struct g2d_buf *g2d_tmp_buf = NULL;
	struct g2d_surface dst_rotate_surface;
	struct g2d_surface dst_blending_surface;
	int ret = 0;

	if (handle == NULL) {
		g2d_printf("%s: Invalid handle!\n", __func__);
		return -1;
	}

	if(!src || !dst)
	{
		g2d_printf("%s: Invalid src and dst parameters!\n", __func__);
		return -1;
	}

	// PXP can't process rotation and alpha blending at the same time.
	// So assemble to 2 steps. First rotation, then alpha blending.
	if(src->rot != G2D_ROTATION_0) {
		// step 1: rotation without alpha blending.
		// fix me, use the largest Bpp(4)
		int size = dst->stride * dst->height * 4;
		g2d_tmp_buf = g2d_alloc(size, 0);
		if(g2d_tmp_buf == NULL) {
			g2d_printf("%s: alloc tmp buf failed, size %d\n", __FUNCTION__, size);
			return -1;
		}

		memcpy(&dst_rotate_surface, dst, sizeof(dst_rotate_surface));
		dst_rotate_surface.planes[0] = g2d_tmp_buf->buf_paddr;
		dst_rotate_surface.format = src->format;

		ret = g2d_blit_wrap(handle, src, &dst_rotate_surface);

		if(ret) {
			g2d_printf("%s: g2d_blit_wrap failed, ret %d\n", __FUNCTION__, ret);
			return ret;
		}

		// prepare step 2: alpha blending
		dst_rotate_surface.blendfunc = src->blendfunc;
		dst_rotate_surface.global_alpha = src->global_alpha;
		dst_rotate_surface.clrcolor = src->clrcolor;
		dst_rotate_surface.rot = G2D_ROTATION_0;

		// copy dst to dst_blending_surface, so the input paramter keep un-change
		memcpy(&dst_blending_surface, dst, sizeof(dst_blending_surface));
		dst_blending_surface.rot = G2D_ROTATION_0;

		src = &dst_rotate_surface;
		dst = &dst_blending_surface;

		ret = g2d_blit_wrap(handle, src, dst);
		if(ret) {
			g2d_printf("%s: g2d_blit_wrap failed, ret %d\n", __FUNCTION__, ret);
			return ret;
		}
		ret = g2d_finish(handle);
		if(ret) {
			g2d_printf("%s: g2d_finish failed, ret %d\n", __FUNCTION__, ret);
			return ret;
		}
		if(g2d_tmp_buf) {
			g2d_free(g2d_tmp_buf);
		}

		return ret;
	}else{
		ret = g2d_blit_wrap(handle, src, dst);

		return ret;
	}
}


int g2d_blitEx(void *handle, struct g2d_surfaceEx *srcEx, struct g2d_surfaceEx *dstEx)
{
    struct g2d_surface *src = (struct g2d_surface *)srcEx;
    struct g2d_surface *dst = (struct g2d_surface *)dstEx;

    return g2d_blit(handle,src,dst);
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

static int g2d_dev_open(int cacheable)
{
    int ret;
    int devfd = -1;
    char device[256];

    if (cacheable)
        ret = snprintf(device, 256, "%s/%s", DEVPATH, HEAPNAME_CACHED);
    else
        ret = snprintf(device, 256, "%s/%s", DEVPATH, HEAPNAME_UNCACHED);

    if (ret < 0) {
        g2d_printf("%s: snprintf failed\n", __FUNCTION__);
        return G2D_STATUS_FAIL;
    }

    devfd = open(device, O_RDWR);
    if (devfd < 0) {
        g2d_printf("%s: open %s failed, %s\n", __FUNCTION__, device,
                   strerror(errno));
        return G2D_STATUS_FAIL;
    }

    return devfd;
}

static int g2d_dev_close(int devfd)
{
    if (devfd < 0) {
        g2d_printf("%s: Invalid argument\n", __FUNCTION__);
        return G2D_STATUS_FAIL;
    }

    if (close(devfd) < 0) {
        g2d_printf("%s: close fd %d failed, %s\n", __FUNCTION__, devfd,
                   strerror(errno));
        return G2D_STATUS_FAIL;
    }

    return G2D_STATUS_OK;
}

static int dmabuf_map(int dmafd, size_t size, void **vaddr)
{
    void *addr;

    addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, dmafd, 0);
    if (addr == MAP_FAILED) {
        g2d_printf("%s: Could not mmap %s", __FUNCTION__, strerror(errno));
        return G2D_STATUS_FAIL;
    }

    *vaddr = addr;
    return G2D_STATUS_OK;
}

static void dmabuf_unmap(void *vaddr, size_t size)
{
    munmap(vaddr, size);
}

static int dmabuf_ioctl(int devfd, int req, void *arg)
{
    if (ioctl(devfd, req, arg) < 0) {
        g2d_printf("%s: ioctl %x failed: %s\n", __FUNCTION__, req,
                   strerror(errno));
        return G2D_STATUS_FAIL;
    }

    return G2D_STATUS_OK;
}

static int dmabuf_get_phys(int dmafd, unsigned long *paddr)
{
    int ret = 0;

    if (!paddr) {
        g2d_printf("%s: Invalid argument\n", __FUNCTION__);
        return G2D_STATUS_FAIL;
    }

#ifdef BUILD_FOR_ANDROID
    struct dmabuf_imx_phys_data data;
    int fd_;
    fd_ = open("/dev/dmabuf_imx", O_RDONLY | O_CLOEXEC);
    if (fd_ < 0) {
        ALOGE("open /dev/dmabuf_imx failed: %s", strerror(errno));
        return -EINVAL;
     }
    data.dmafd = dmafd;
    if (ioctl(fd_, DMABUF_GET_PHYS, &data) < 0) {
        ALOGE("%s DMABUF_GET_PHYS  failed",__func__);
        close(fd_);
        return -EINVAL;
    } else
        *paddr = data.phys;

    close(fd_);
#else
    struct dma_buf_phys query;
    ret = dmabuf_ioctl(dmafd, DMA_BUF_IOCTL_PHYS, &query);
    *paddr = query.phys;
#endif

    return ret;
}

static int dmabuf_heap_alloc_fdflags(int devfd, size_t len, unsigned int devfd_flags,
                                     unsigned int heap_flags, int *dmafd)
{
    struct dma_heap_allocation_data query = {
        .len = len,
        .fd = 0,
        .fd_flags = devfd_flags,
        .heap_flags = heap_flags,
    };
    int ret;

    if (!dmafd) {
        g2d_printf("%s: Invalid argument\n", __FUNCTION__);
        return G2D_STATUS_FAIL;
    }

    ret = dmabuf_ioctl(devfd, DMA_HEAP_IOCTL_ALLOC, &query);
    *dmafd = (int)query.fd;

    return ret;
}

static int dmabuf_heap_alloc(int devfd, size_t len, unsigned int flags,
                             int *dmafd)
{
    return dmabuf_heap_alloc_fdflags(devfd, len, O_RDWR | O_CLOEXEC, flags, dmafd);
}

struct g2d_buf *g2d_alloc(int size, int cacheable)
{
    struct g2d_buf *buf;
    struct dma_buf * dma_buf_ptr = NULL;
    size_t alignedSize = __PAGE_ALIGN(size);

    dma_buf_ptr = (struct dma_buf *)calloc(1, sizeof(struct dma_buf));

    if (dmabuf_heap_alloc(devfd, alignedSize, 0, &dma_buf_ptr->dmafd)) {
        g2d_printf("%s: dmabuf heap alloc failed\n", __FUNCTION__);
        return NULL;
    }

    if (dmabuf_get_phys(dma_buf_ptr->dmafd, &dma_buf_ptr->paddr)) {
        g2d_printf("%s: dmabuf get phys failed\n", __FUNCTION__);
        goto err_close;
    }

    if (dmabuf_map(dma_buf_ptr->dmafd, alignedSize, &dma_buf_ptr->vaddr)) {
        g2d_printf("%s: dmabuf map failed\n", __FUNCTION__);
        goto err_close;
    }

    dma_buf_ptr->size = alignedSize;

    buf = (struct g2d_buf *)calloc(1, sizeof(struct g2d_buf));
    if (!buf) {
        g2d_printf("%s: calloc g2d_buf failed\n", __FUNCTION__);
        goto err_unmap;
    }

    buf->buf_handle = (void *)dma_buf_ptr;
    buf->buf_vaddr = dma_buf_ptr->vaddr;
    buf->buf_paddr = (int)dma_buf_ptr->paddr;
    buf->buf_size = (int)dma_buf_ptr->size;

    return buf;

err_unmap:
    dmabuf_unmap(dma_buf_ptr->vaddr, alignedSize);
err_close:
    close(dma_buf_ptr->dmafd);
    free(dma_buf_ptr);

    return NULL;
}

int g2d_free(struct g2d_buf *buf)
{
    struct dma_buf * dma_buf_ptr = NULL;

    if (!buf || !buf->buf_handle) {
        g2d_printf("%s: Invalid argument\n", __FUNCTION__);
        return G2D_STATUS_FAIL;
    }

    dma_buf_ptr = (struct dma_buf *)buf->buf_handle;
    if(dma_buf_ptr->dmafd >= 0) {
        dmabuf_unmap(dma_buf_ptr->vaddr, dma_buf_ptr->size);
        close(dma_buf_ptr->dmafd);
    }
    free(dma_buf_ptr);
    free(buf);

    return G2D_STATUS_OK;
}

struct g2d_buf * g2d_buf_from_fd(int fd)
{
	struct dma_buf *dma_buf_ptr = NULL;
	struct g2d_buf *buf = NULL;
	int ret;

	dma_buf_ptr = (struct dma_buf *)calloc(1, sizeof(struct dma_buf));
	dma_buf_ptr->dmafd = -1;

	ret = dmabuf_get_phys(fd, &dma_buf_ptr->paddr);
	if(ret) {
		g2d_printf("%s: dmabuf get phys failed\n", __FUNCTION__);
		free(dma_buf_ptr);
		return NULL;
	}

	/* Construct g2d_buf */
	buf = (struct g2d_buf *)calloc(1, sizeof(struct g2d_buf));
	if(!buf)
	{
		g2d_printf("%s: Invalid g2d_buf !\n", __FUNCTION__);
		free(dma_buf_ptr);
		return NULL;
	}

	buf->buf_paddr = dma_buf_ptr->paddr;
	buf->buf_handle = (void *)dma_buf_ptr;

	return buf;
}

int g2d_buf_export_fd(struct g2d_buf *buf)
{
	struct dma_buf *dma_buf_ptr = (struct dma_buf *)buf->buf_handle;

	if (dma_buf_ptr->dmafd < 0) {
		g2d_printf("%s: Invalid fd\n", __FUNCTION__);
		return -EINVAL;
	}

	return dma_buf_ptr->dmafd;
}

struct g2d_buf *g2d_buf_from_virt_addr(void *vaddr, int size) {
	g2d_printf("%s: NOT SUPPORTED\n", __FUNCTION__);
	return NULL;
}

int g2d_create_fence_fd(void *handle)
{
	return -1;
}

int g2d_set_warp_coordinates(void *handle, struct g2d_warp_coordinates *coord)
{
    /* Not supported by GPU 2D */

    return -1;
}
