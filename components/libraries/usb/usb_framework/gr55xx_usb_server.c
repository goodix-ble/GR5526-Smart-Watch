/*
 * Copyright (C) 2021, Shenzhen Goodix Technology Co., Ltd.
 * All Rights Reserved.
 */
#include "grx_hal.h"
#include "gr55xx_usb_server.h"
#include "string.h"
#include "stdlib.h"

usb_handle_t g_usb_handle;
struct _gr55xx_usb_request g_reqs[GM_USBD_REQS];

static gr55xx_dev_usb_t usbd =
        {
            .gadget =
            {
                .name = "usbd",
                .ep0 = &usbd.ep[0].ep,
            },
            .ep[0] =
            {
                .num = 0,
                .ep = {
                    .name = "ep0",
                    .maxpacket = USB_EP0_MAX_PKS,
                },
                .req = &g_reqs[0],
            },
            .ep[1] =
            {
                .num = 1,
                .ep = {
                    .name = "ep1",
                    .address = USB_EP1,
                    .maxpacket = USB_EP1_MAX_PKS,
                },
                .req = &g_reqs[1],
            },
            .ep[2] =
            {
                .num = 2,
                .ep = {
                    .name = "ep2",
                    .address = USB_EP2,
                    .maxpacket = USB_EP2_MAX_PKS,
                },
                .req = &g_reqs[2],
            },
            .ep[3] =
            {
                .num = 3,
                .ep = {
                    .name = "ep3",
                    .address = USB_EP3,
                    .maxpacket = USB_EP3_MAX_PKS,
                },
                .req = &g_reqs[3],
            },
            .ep[4] =
            {
                .num = 4,
                .ep = {
                    .name = "ep4",
                    .address = USB_EP4,
                    .maxpacket = USB_EP4_MAX_PKS,
                },
                .req = &g_reqs[4],
            },
            .ep[5] =
            {
                .num = 5,
                .ep = {
                    .name = "ep5",
                    .address = USB_EP5,
                    .maxpacket = USB_EP5_MAX_PKS,
                },
                .req = &g_reqs[5],
            },
        };


static inline void usb_set_cmd_ok(void)
{
    /* cmd ok */
    hal_usb_set_cmd_ok(&g_usb_handle);

    USB_LOG("%s\r\n",__FUNCTION__);

    return;
}

static inline void usb_set_cmd_err(void)
{
    /* cmd error */
    hal_usb_set_cmd_err(&g_usb_handle);

    USB_LOG("%s\r\n",__FUNCTION__);

    return;
}


static void usb_ep_done(struct _gr55xx_usb_ep *ep, struct _gr55xx_usb_request *req, int status)
{
    unsigned halted = ep->halted;

    if(!list_empty(&req->entry))
    {
        list_del(&req->entry);
    }

    if (req->req.status == GR_DRV_USB_ERROR_INPROGRESS)
        req->req.status = status;
    else
        status = req->req.status;

    ep->halted = 1;
    if (req->req.complete)
        req->req.complete(&ep->ep, &req->req);
    ep->halted = halted;

    return;
}

static int usb_ep_fifo_write(int ep, struct _gr55xx_usb_request *req, unsigned max)
{
    unsigned len = min(req->req.length - req->req.actual, max);
    uint8_t *buf = req->req.buf + req->req.actual;

    USB_LOG("usb_ep_fifo_write EP%d  already send:%d  buflen:%d  thislen:%d \r\n",ep,req->req.actual, req->req.length, len);

    if (len)
    {
        int32_t loops = len;
        uint32_t value;
        uint8_t *pdata = (uint8_t*)buf;
        while (loops > 0)
        {
            if (ep == USB_EP3)
            {
                hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP3,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_DEFAULT);
                pdata += 4;
                loops -= 4;
            }
            else if(ep == USB_EP4)
            {
                if(loops >= 4)
                {
                    hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_4BYTE);
                    pdata += 4;
                    loops -= 4;
                }
                else
                {   switch(loops)
                    {
                        case 1:
                            hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_1BYTE);
                            break;
                        case 2:
                            hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_2BYTE);
                            break;
                        case 3:
                            hal_usb_write_ep_fifo(&g_usb_handle,HAL_USB_EP4,*((uint32_t*)pdata),USB_EP4_FIFO_WEN_3BYTE);
                            break;
                        default:
                            break;
                    }
                    loops = 0;
                }
            }
            else //EP0 EP2
            {
                value = (uint32_t)(*pdata);
                USB_LOG("usb_ep_fifo_write EP[%d] the %d value is: %x \r\n", ep,loops, value);
                hal_usb_write_ep_fifo(&g_usb_handle, (hal_usb_ep_t)ep, value, USB_EP4_FIFO_WEN_DEFAULT);
                pdata ++;
                loops--;
            }
        }
    }

    req->req.actual += len;

    return len;
}

static int usb_ep_fifo_read(int ep, uint8_t *buf, struct _gr55xx_usb_request *req, unsigned avail)
{
    unsigned len = min(req->req.length - req->req.actual, avail);

    if (len)
    {
        uint32_t *pdata = (uint32_t *)buf;
        uint32_t loops = ((len + 3) >> 2);

        while (loops)
        {
            *pdata++ = hal_usb_read_ep_fifo(&g_usb_handle,(hal_usb_ep_t)ep);
            loops--;
        }
    }

    req->req.actual += len;

    return len;
}

static void usb_ep_dma_write(int ep, struct _gr55xx_usb_request *req)
{
    uint8_t *buf = req->req.buf + req->req.actual;
    unsigned sz = min(req->req.length - req->req.actual, USB_XFER_LEN_MAX);

    if(ep == USB_EP3 || ep == USB_EP4)
    {
        USB_LOG("[usb_ep[%d]_dma_write epbuf:%p nowbuf:%p size:%d length:%d  actual:%d\r\n",ep,req->req.buf, buf, sz, req->req.length, req->req.actual);
        hal_status_t res = HAL_OK;
        if(ep == USB_EP3)
        {
            if(ll_usb_is_enabled_ep3_ahb_m(USB))
            {
                USB_LOG("WARRING: EP3 DMA IS BUSY. Wait\r\n");
                while(ll_usb_is_enabled_ep3_ahb_m(USB));
            }
        }
        else //USB_EP4
        {
            if(ll_usb_is_enabled_ep4_ahb_m(USB))
            {
                USB_WARRING("WARRING: EP4 DMA IS BUSY. Wait\r\n");
                while(ll_usb_is_enabled_ep4_ahb_m(USB));
            }
//            __HAL_USB_SET_EP4_BURST_SIZE(&g_usb_handle, sz);
        }

        res = hal_usb_ep_transmit_dma(&g_usb_handle,(hal_usb_ep_t)ep,buf,sz);
        if(res == HAL_OK)
        {
            req->req.actual += sz;
        }
        else if(res == HAL_BUSY)
        {
            USB_WARRING("WARRING: usb_ep[%d] BUSY \r\n",ep);
        }
        else
        {
            USB_WARRING("WARRING: usb_ep[%d] HAL_ERROR \r\n ",ep);
        }
    }
}

static int usb_ep_write(struct _gr55xx_usb_ep *ep, struct _gr55xx_usb_request *req)
{
    int is_last;
    unsigned count, maxpacket;
    uint32_t idx;

    idx = ep->bEndpointAddress & 0x7F;

    hal_status_t ret;
    ret = hal_usb_ep_write_start(&g_usb_handle,(hal_usb_ep_t)idx);
    if(HAL_OK != ret)
    {
        USB_ERROR("ERROR: hal_usb_ep_write_start ret %d \r\n",ret);
    }

    maxpacket = ep->ep.maxpacket;
    count = usb_ep_fifo_write(idx, req, maxpacket);

    /*MCU has write all data to IN FIFO, then inform USB controller */
    hal_usb_ep_write_end(&g_usb_handle,(hal_usb_ep_t)idx);

    if (count != maxpacket)
        is_last = 1;
    else if (req->req.length != req->req.actual)
        is_last = 0;
    else
        is_last = 2;

    if (idx == 0)
    {
        USB_LOG("Written ep%d %d.%d of %d b [last %d,z %d] \r\n",
                    idx, count, req->req.actual, req->req.length,
                            is_last, req->req.zero);
    }

    if (is_last)
    {
        if (idx == 0) {
            ep->dev->ep0state = EP0_IDLE;
            usb_set_cmd_ok();
        }

        usb_ep_done(ep, req, GR_DRV_USB_OK);
        is_last = 1;
    }

    return is_last;
}

static int usb_ep_read(struct _gr55xx_usb_ep *ep, struct _gr55xx_usb_request *req)
{
    int is_last;
    uint32_t status, idx;
    uint32_t timeout = 100000;
    uint8_t *buf;
    unsigned avail, bufferspace, count;

    idx = ep->bEndpointAddress & 0x7F;

    status = hal_usb_ep_read_start(&g_usb_handle,(hal_usb_ep_t)idx);

    if (status != HAL_OK)
    {
        return 0;
    }

    if((idx == 0) && (ep->dev->ep0state == EP0_OUT_DATA_PHASE) && (req->req.length > 0))
    {
        usb_set_cmd_ok();

        while(!(__HAL_USB_GET_EP0_OUT_DAT_RDY(&g_usb_handle)) && (--timeout) > 0);
    }

    buf = req->req.buf + req->req.actual;

    avail = 0;
    if (req->req.length)
    {
        bufferspace = req->req.length - req->req.actual;
        if (!bufferspace) {

            USB_ERROR("buffer full!");

            return GR_DRV_USB_ERROR;
        }

        if(idx == USB_EP0)
        {
            count = hal_usb_get_ep0_rx_data_sum(&g_usb_handle);
        }
        else
        {
            count = hal_usb_get_ep1_rx_data_sum(&g_usb_handle);
        }

        if (count > ep->ep.maxpacket)
            avail = ep->ep.maxpacket;
        else
            avail = count;
    }

    count = usb_ep_fifo_read(idx, buf, req, avail);

    /*MCU has receive all data from OUT FIFO, then clear out packet ready*/
    hal_usb_ep_read_end(&g_usb_handle,(hal_usb_ep_t)idx);

    if (idx != 0 && count < ep->ep.maxpacket) {
        is_last = 1;
        if (count != avail)
            req->req.status = GR_DRV_USB_ERROR_OVERFLOW;
    } else {
        is_last = (req->req.length <= req->req.actual) ? 1 : 0;
    }

    if (idx == 0)
    {
        USB_LOG("fifo addr:0x%p count:%d [is_last:%d] \r\n", buf, count, is_last);
    }

    if (is_last)
    {
        if (idx == 0) {
#if 0
            if (req->req.zero == 0) {
                ep->dev->ep0state = EP0_IDLE;
                usb_set_cmd_ok();
            }
#endif
            ep->dev->ep0state = EP0_IDLE;
            usb_set_cmd_ok();
        }

        usb_ep_done(ep, req, 0);
        is_last = 1;
    }

    return is_last;
}


static void usb_handle_ep0_idle(struct _gr55xx_usb_ep *ep, struct usb_ctrlrequest *setup)
{
    int i;
    int ret, tmp;

    uint32_t* ptr = (uint32_t *)setup;

    gr55xx_dev_usb_t *udc = &usbd;

    for (i = 0; i < 2; i++)
    {
        ptr[i] = hal_usb_read_ep_fifo(&g_usb_handle,HAL_USB_EP0);
    }

    USB_LOG("usb_handle_ep0_idle [%d] %02x %02x %04x %04x %04x \r\n", hal_usb_get_ep0_rx_data_sum(&g_usb_handle),setup->bRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);

    udc->req_config = 0;

    if ((setup->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
    {
        switch (setup->bRequest) {
            case USB_REQ_SET_CONFIGURATION:
                USB_LOG( "usb request set configuration ...\r\n");
                if (setup->bRequestType == USB_RECIP_DEVICE) {
                    udc->req_config = 1;
                    if (udc->state == DEVSTATE_DEFAULT)
                    {
                        USB_LOG("set configration error!\r\n");
                        usb_set_cmd_err();
                        return;
                    }
                    else
                    {
                        udc->config = setup->wValue & 0xff;
                        if (udc->config == 0)
                        {
                            udc->state = DEVSTATE_ADDRESS;
                        }
                        else
                        {
                            udc->state = DEVSTATE_CONFIGFS;
                            __HAL_USB_ENABLE_CFG_STAT(&g_usb_handle);
                        }
                    }
                }
                break;
            case USB_REQ_SET_ADDRESS:
                USB_LOG( "usb request set address ...\r\n");
                if (setup->bRequestType == USB_RECIP_DEVICE) {
                    tmp = setup->wValue & 0x7F;
                    USB_LOG( "usb request set address,host set address value is: %d  ...\r\n",tmp);
                    hal_usb_set_addr(&g_usb_handle,tmp);
                    udc->state = DEVSTATE_ADDRESS;
                    udc->address = tmp;
                    usb_set_cmd_ok();
                    return;
                }
                break;
            case USB_REQ_GET_STATUS:
                USB_LOG( "usb request get status ...\r\n");
                if ((setup->bRequestType & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
                    __HAL_USB_CLEAR_EP0_IFIFO(&g_usb_handle);
                    hal_usb_set_addr(&g_usb_handle,udc->state);
                    usb_set_cmd_ok();
                    return;
                }
                break;
            case USB_REQ_CLEAR_FEATURE:
                usb_set_cmd_ok();
                USB_LOG( "usb request clear feature ...\r\n");
                __HAL_USB_ENABLE_DEV_REMOTE_WAKEUP(&g_usb_handle);
                break;
            case USB_REQ_SET_FEATURE:
                usb_set_cmd_ok();
                USB_LOG( "usb request set feature ...\r\n");
                __HAL_USB_DISABLE_DEV_REMOTE_WAKEUP(&g_usb_handle);
                //udelay(1000*5000);//5s
                //regs->usb_ctrl |= MCU_WAKEUP_MASK;
                break;
            default:
                break;
        }
    }

    if (setup->bRequestType & USB_DIR_IN)
        udc->ep0state = EP0_IN_DATA_PHASE;
    else
        udc->ep0state = EP0_OUT_DATA_PHASE;

    if (!udc->composite_driver)
        return;

    /* deliver the request to the gadget driver */
    ret = udc->composite_driver->setup(&udc->gadget, setup);
    if (ret < 0) {
        if (udc->req_config) {
            USB_LOG("config change %02x fail %d?\r\n", setup->bRequest, ret);
            usb_set_cmd_err();
            return;
        }

        if (ret == GR_DRV_USB_ERROR_NOTSUPP)
        {
            USB_LOG("\r\n operation not supported.\r\n");
        }
        else
        {
            USB_LOG("dev->driver->setup failed. (%d)\r\n", ret);
        }

        udc->ep0state = EP0_IDLE;
        usb_set_cmd_ok();
    }

    USB_LOG("ep0state %d \r\n ", udc->ep0state);

    return;
}

static void usb_handle_ep0(void)
{
    struct usb_ctrlrequest setup;
    gr55xx_dev_usb_t *udc = &usbd;
    struct _gr55xx_usb_ep *ep = &(udc->ep[0]);
    struct _gr55xx_usb_request *req = NULL;

    if (!list_empty(&ep->queue))
    {
        req = list_entry(ep->queue.next, struct _gr55xx_usb_request, entry);
    }

    switch (udc->ep0state) {
        case EP0_IDLE:
            usb_handle_ep0_idle(ep, &setup);
            break;

        case EP0_OUT_DATA_PHASE:        /* SET_DESCRIPTOR etc */
            USB_LOG("ep0 out data phase ... what now? \r\n");
            if (req){
                if (usb_ep_read(ep, req)){
                    usb_set_cmd_ok();
                    udc->ep0state = EP0_IDLE;
                }
            }
            break;
        default:
            USB_LOG("usb_handle_ep0 default udc->ep0state= %d \r\n",udc->ep0state);
            break;
    }

    return;
}

static void usb_handle_ep(struct _gr55xx_usb_ep *ep)
{
    struct _gr55xx_usb_request *req = NULL;
    int is_in = ep->bEndpointAddress & USB_DIR_IN;

    USB_LOG("runing usb_handle_ep ep%d \r\n", ep->num);

    if (!list_empty(&ep->queue))
    {
        req = list_entry(ep->queue.next, struct _gr55xx_usb_request, entry);
    }

    if (is_in)
    {
        if (req)
        {
            int idx = ep->bEndpointAddress & 0x7f;
            if(idx == USB_EP3 || idx == USB_EP4)
            {
                do {
                    if(req->req.actual == req->req.length)
                    {
                        usb_ep_done(ep, req, 0);
                        break;
                    }

                    if (list_empty(&ep->queue))
                    {
                        break;
                    }

                    USB_LOG("EP%d dma continue...",idx);

                    req = list_entry(ep->queue.next, struct _gr55xx_usb_request, entry);
                    usb_ep_dma_write(idx, req);
                } while (0);
            }
            else
            {
                usb_ep_write(ep, req);
                USB_LOG("EP%d usb_ep_write\r\n",idx);
            }
        }
    }
    else
    {
        if (req)
        {
            USB_LOG("usb_ep_read(ep[%p], req:0x%p)!\n",ep, req);
            usb_ep_read(ep, req);
        }
    }

    return;
}

void hal_usb_host_reset_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usbd.ep0state = EP0_IDLE;
}

void hal_usb_suspend_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usbd.ep0state = EP0_IDLE;
}

void hal_usb_into_config_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    struct usb_ctrlrequest setup;

    setup.bRequestType = 0x00;
    setup.bRequest = 0x09;
    setup.wValue = 0x0001;
    setup.wIndex = 0x00;
    setup.wLength = 0x00;
    usbd.req_config = 1;
    usbd.state = DEVSTATE_CONFIGFS;
    usbd.ep0state = EP0_OUT_DATA_PHASE;
    usbd.composite_driver->setup(&usbd.gadget, &setup);
    usbd.ep0state = EP0_IDLE;
}

void hal_usb_ep0_out_ready_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usb_handle_ep0();
}

void hal_usb_ep1_out_ready_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usbd.composite_driver->ep_callback(&usbd.gadget, &usbd.ep[USB_EP1].ep);
}

void hal_usb_ep2_tx_done_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
}

void hal_usb_sof_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
}
void hal_usb_ep0_tx_done_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
}

void hal_usb_ep3_ahb_xfer_done_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
}

void hal_usb_ep3_tx_done_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usb_handle_ep(&(usbd.ep[USB_EP3]));
}

#if USB_EP4_DMA_MODE_EN
void hal_usb_ep4_ahb_xfer_done_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usbd.composite_driver->ep_callback(&usbd.gadget, &usbd.ep[USB_EP4].ep);
}
#else
void hal_usb_ep4_tx_done_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usb_handle_ep(&(usbd.ep[USB_EP4]));
}
#endif

#if USB_EP5_DMA_MODE_EN
void hal_usb_ep5_ahb_xfer_done_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    __HAL_USB_SET_EP5_TIMER_VAL(p_usb,3);
    usbd.ep[USB_EP5].req->req.actual = hal_usb_get_ep5_rx_data_sum(p_usb);
    /*EP5 buffer data must be copy in 1ms to avoid overwriting*/
    usbd.composite_driver->ep_callback(&usbd.gadget, &usbd.ep[USB_EP5].ep);
    /*EP5 DMA enable should after EP5 buffer data having been copyed*/
    __HAL_USB_ENABLE_EP5_DMA_READ(p_usb);
}
#else
void hal_usb_ep5_out_ready_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    usbd.composite_driver->ep_callback(&usbd.gadget, &usbd.ep[USB_EP5].ep);
}
#endif

void hal_usb_ep5_timer_out_err_callback(usb_handle_t *p_usb)
{
    USB_LOG("%s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_EP5_TIMER;
    __HAL_USB_SET_EP5_TIMER_VAL(p_usb,0);
    usbd.ep[USB_EP5].req->req.actual = hal_usb_get_ep5_rx_data_sum(p_usb);
    /*EP5 buffer data must be copy in 1ms to avoid overwriting*/
    usbd.composite_driver->ep_callback(&usbd.gadget, &usbd.ep[USB_EP5].ep);
    /*EP5 DMA enable should after EP5 buffer data having been copyed*/
    __HAL_USB_ENABLE_EP5_DMA_READ(p_usb);
}

void hal_usb_crc16_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_CRC16;
}

 void hal_usb_upid_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_UPID;
}

void hal_usb_time_out_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_TIMEOUT;
}

void hal_usb_seq_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_SEQ;
}

void hal_usb_pid_cks_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_PID_CKS;
}

void hal_usb_pid_crc_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_PID_CRC;
}

void hal_usb_ahb_xfer_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_DMA_RX;
}

void hal_usb_nse_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_NSE;
}

void hal_usb_sync_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_SYNC;
}

void hal_usb_bit_stuff_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_BIT_STUFF;
}

void hal_usb_byte_err_callback(usb_handle_t *p_usb)
{
    USB_ERROR("ERROR: %s\r\n",__FUNCTION__);
    p_usb->error_code = HAL_USB_STATE_ERROR_BYTE;
}

struct usb_request *gr55xx_drv_usbd_alloc_request(struct usb_ep *ep)
{
    struct _gr55xx_usb_request *req;
    gr55xx_dev_usb_t *udc = &usbd;

    if(list_empty(&udc->head) || !ep)
    {
        return NULL;
    }

    req =  udc->ep[ep->address].req;
    memset(&req->req, 0x0, sizeof(struct usb_request));

    return &req->req;
}

void gr55xx_drv_usbd_free_request(struct usb_ep *ep, struct usb_request *req)
{
    gr55xx_dev_usb_t *udc = &usbd;
    struct _gr55xx_usb_request *_req = list_entry(req, struct _gr55xx_usb_request, req);

    INIT_LIST_HEAD(&_req->entry);
    list_add_tail(&_req->entry, &udc->head);

    return;
}

gr55xx_usb_state_t gr55xx_drv_usbd_enable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc)
{
    uint8_t type;
    int id, xfer;
    struct _gr55xx_usb_ep *_ep = list_entry(ep, struct _gr55xx_usb_ep, ep);

   if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT)
   {
       USB_LOG("%s not enabled.", ep ? _ep->ep.name : NULL);
       return GR_DRV_USB_ERROR;
   }

    _ep->ep.desc = desc;
    _ep->halted = 0;
    _ep->bEndpointAddress = desc->bEndpointAddress;

    //setup transfer type
    type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

    xfer = (type == USB_ENDPOINT_XFER_BULK)?(2):((type==USB_ENDPOINT_XFER_ISOC)?(1):((type==USB_ENDPOINT_XFER_INT)?(0):(2)));

    USB_LOG("\r\nep%d xfer type is %d \n", _ep->bEndpointAddress&0x7f, xfer);

    id = desc->bEndpointAddress & 0x7f;
      switch(id)
    {
        case USB_EP1:
            hal_usb_set_and_unhalt_ep(&g_usb_handle,HAL_USB_EP1,xfer);
            break;
        case USB_EP2:
            hal_usb_set_and_unhalt_ep(&g_usb_handle,HAL_USB_EP2,xfer);
            break;
        case USB_EP3:
            hal_usb_set_and_unhalt_ep(&g_usb_handle,HAL_USB_EP3,xfer);
            break;
        case USB_EP4:
#if USB_EP4_DMA_MODE_EN
            ll_usb_enable_it(g_usb_handle.p_instance, LL_USB_INT_EN_EP4_AHB_XFER_DONE);
            ll_usb_disable_ep4_ahb_m(g_usb_handle.p_instance);
#else
            ll_usb_enable_it(g_usb_handle.p_instance, LL_USB_INT_EN_EP4_TX_DONE);
            ll_usb_disable_ep4_dat_rdy(g_usb_handle.p_instance);
#endif
            ll_usb_enable_clr_ep4_fifo(USB);
            hal_usb_set_and_unhalt_ep(&g_usb_handle,HAL_USB_EP4,xfer);
            break;
        case USB_EP5:
#if USB_EP5_DMA_MODE_EN
            __HAL_USB_SET_EP5_TIMER_VAL(&g_usb_handle,0);
            __HAL_USB_DISABLE_IT(&g_usb_handle,USB_IT_EP5_OUT_READY);
            __HAL_USB_ENABLE_IT(&g_usb_handle,USB_IT_EP5_AHB_XFER_DONE | USB_IT_EP5_TIMER_OUT_ERR);
            ll_usb_enable_ep5_fifo_clr(USB);
            __HAL_USB_ENABLE_EP5_DMA_READ(&g_usb_handle);
#else
            __HAL_USB_ENABLE_IT(&g_usb_handle,USB_IT_EP5_OUT_READY);
#endif
            hal_usb_set_and_unhalt_ep(&g_usb_handle,HAL_USB_EP5,xfer);
            break;
        default:
            break;
    }

    return GR_DRV_USB_OK;
}

gr55xx_usb_state_t gr55xx_drv_usbd_disable(struct usb_ep *ep)
{
    int id;

    struct _gr55xx_usb_ep *_ep = list_entry(ep, struct _gr55xx_usb_ep, ep);

//    if (!ep || !_ep->ep.desc) {
//        USB_ERROR("%s not disable. return GR_DRV_USB_ERROR \r\n ", ep ? _ep->ep.name : NULL);
//        return GR_DRV_USB_ERROR;
//    }

    _ep->ep.desc = NULL;
    _ep->halted = 1;

    id = _ep->bEndpointAddress & 0x7f;
     switch(id)
    {
        case USB_EP1:
            hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP1);
            break;
        case USB_EP2:
            hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP2);
            break;
        case USB_EP3:
            hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP3);
            break;
        case USB_EP4:
            if ((ll_usb_is_enabled_ep4_ahb_m(g_usb_handle.p_instance)))
            {
               USB_ERROR("%s ERROR: EP4 DMA is busy, halt EP4 FORCE \r\n", __FUNCTION__);
            }
            hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP4);
            break;
        case USB_EP5:
#if USB_EP5_DMA_MODE_EN
            __HAL_USB_SET_EP5_TIMER_VAL(&g_usb_handle,1);
            __HAL_USB_DISABLE_IT(&g_usb_handle,USB_IT_EP5_AHB_XFER_DONE  | USB_IT_EP5_TIMER_OUT_ERR | USB_IT_EP5_OUT_READY);
#else
            __HAL_USB_DISABLE_IT(&g_usb_handle,USB_IT_EP5_OUT_READY);
#endif
             ll_usb_enable_ep5_fifo_clr(USB);
            __HAL_USB_DISABLE_EP5_DMA_READ(&g_usb_handle);
            hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP5);
            break;
        default:
            break;
    }


    return GR_DRV_USB_OK;
}


gr55xx_usb_state_t gr55xx_drv_usbd_queue(struct usb_ep *ep, struct usb_request *req)
{
    gr55xx_usb_state_t err = GR_DRV_USB_OK;
    struct _gr55xx_usb_ep *_ep = container_of(ep, struct _gr55xx_usb_ep, ep);
    struct _gr55xx_usb_request *_req = list_entry(req, struct _gr55xx_usb_request, req);
    struct _gr55xx_dev_usb *usb_dev = _ep->dev;

    if (!ep)
    {
        USB_ERROR("ERROR: invalid args \r\n");
        return GR_DRV_USB_ERROR;
    }

    if(!usb_dev || !usb_dev->composite_driver)
    {
        USB_ERROR("ERROR: invalid args \r\n");
        return GR_DRV_USB_ERROR;
    }

    if (!req || !req->complete)
    {
        USB_ERROR("ERROR: invalid args \r\n");
        return GR_DRV_USB_ERROR;
    }

    req->status = GR_DRV_USB_ERROR_INPROGRESS;
    req->actual = 0;

    USB_LOG("gr55xx_drv_usbd_queue ep%x len %d list_empty(&_ep->queue)is %d, &_ep->queue is :%p \r\n", _ep->bEndpointAddress, req->length,list_empty(&_ep->queue),&_ep->queue);
    int idx = _ep->bEndpointAddress & 0x7f;
    if (list_empty(&_ep->queue) && !_ep->halted)
    {
        if (_ep->bEndpointAddress == 0 /* ep0 */)
        {
            switch (usb_dev->ep0state)
            {
                case EP0_IN_DATA_PHASE:
                    while(req->status == GR_DRV_USB_ERROR_INPROGRESS)
                    {
                        usb_ep_write(_ep, _req);
                    }
                    _req = NULL;
                    break;
                case EP0_OUT_DATA_PHASE:
                    if (usb_ep_read(_ep, _req))
                    {
                        _req = NULL;
                    }
                    break;
                default:
                    USB_ERROR("ERROR: %s return GR_DRV_USB_ERROR_HALT \r\n",__FUNCTION__);
                    return GR_DRV_USB_ERROR_HALT;
            }
        }
        else if (_ep->bEndpointAddress & USB_DIR_IN)
        {
            //EP3 FIFO write 4byte every time
            if (((idx == USB_EP3) && (_req->req.length >= _ep->ep.maxpacket))
                ||((idx == USB_EP3) && (_req->req.length < _ep->ep.maxpacket) && (_req->req.length % 4 != 0)))
            {
                usb_ep_dma_write(idx, _req);
                _req = NULL;
            }
#if USB_EP4_DMA_MODE_EN
            else if(idx == USB_EP4)
            {
                usb_ep_dma_write(idx, _req);
                _req = NULL;
            }
#endif
            else
            {
                //EP0 EP2
                while(req->status == GR_DRV_USB_ERROR_INPROGRESS)
                {
                    usb_ep_write(_ep, _req);
                }
                _req = NULL;
            }
        }
        else if (usb_ep_read(_ep, _req))
        {
            //polling + interrrupt mode
            _req = NULL;
        }
    }
    else if(idx == USB_EP4 && !_ep->halted)
    {
#if USB_EP4_DMA_MODE_EN
        usb_ep_dma_write(idx, _req);
        _req = NULL;
#else
        while(req->status == GR_DRV_USB_ERROR_INPROGRESS)
        {
            usb_ep_write(_ep, _req);
        }
        _req = NULL;
#endif
    }

    if (_req)
    {
        list_add_tail(&_req->entry, &_ep->queue);
    }

    return err;
}

gr55xx_usb_state_t gr55xx_drv_usbd_set_halt(struct usb_ep *ep, int value)
{
    int id;
    struct _gr55xx_usb_ep *_ep = list_entry(ep, struct _gr55xx_usb_ep, ep);

    _ep->halted = value ? 1 : 0;
    id = _ep->bEndpointAddress & 0x7f;

    switch(id)
    {
        case USB_EP1:
            if(!_ep->halted)
            {
                hal_usb_un_halt_ep(&g_usb_handle,HAL_USB_EP1);
            }
            else
            {
                hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP1);
            }
            break;
        case USB_EP2:
            if(!_ep->halted)
            {
                hal_usb_un_halt_ep(&g_usb_handle,HAL_USB_EP2);
            }
            else
            {
                hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP2);
            }
            break;
        case USB_EP3:
            if(!_ep->halted)
            {
                hal_usb_un_halt_ep(&g_usb_handle,HAL_USB_EP3);
            }
            else
            {
                hal_usb_halt_ep(&g_usb_handle,HAL_USB_EP3);
            }
            break;
        default:
            break;
    }

    return GR_DRV_USB_OK;
}

gr55xx_usb_state_t gr55xx_drv_usbd_register_gadget_driver(struct usb_gadget_driver *driver)
{
    int i, retval;

    gr55xx_dev_usb_t *udc = &usbd;

    udc->ep0state = EP0_IDLE;

    INIT_LIST_HEAD(&udc->gadget.ep_list);
    for (i = 0; i < ENDPOINT_NUM; i++)
    {
        struct _gr55xx_usb_ep *ep = &udc->ep[i];

        if (i != 0)
        {
            list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
        }
        ep->dev = udc;
        ep->ep.desc = NULL;

        INIT_LIST_HEAD(&ep->queue);
    }

    udc->composite_driver = driver;

    if ((retval = driver->bind(&udc->gadget)) != 0)// ref to composite bind interface
    {
        USB_LOG("usbd bind composite driver error!\r\n");
    }

    return (gr55xx_usb_state_t)retval;
}

uint32_t gr55xx_drv_usbd_driver_data(struct usb_ep *ep)
{
    uint32_t status,count,avail;
    uint8_t *pdata;
    uint32_t *prxdata;

    if((ep->address == USB_EP1) || (ep->address == USB_EP5))
    {
        status = hal_usb_ep_read_start(&g_usb_handle,(hal_usb_ep_t)ep->address);

        if (status != HAL_OK)
        {
            return 0;
        }

        if(ep->address == USB_EP1)
        {
            count =  hal_usb_get_ep1_rx_data_sum(&g_usb_handle);
        }
        else if(ep->address == USB_EP5)
        {
            count =  hal_usb_get_ep5_rx_data_sum(&g_usb_handle);
        }
        else
        {
            USB_LOG("%s, %d \r\n",__FILE__,__LINE__);;
        }

        USB_LOG(" \r\n gr55xx_drv_usbd_driver_data count value  is: %d \r\n",count);

        if (count > ep->maxpacket)
        {
            avail = ep->maxpacket;
        }
        else
        {
            avail = count;
        }

        status = avail;
        pdata = (uint8_t *)ep->driver_data;

        avail = ((avail + 3) >> 2);
        prxdata = (uint32_t *)pdata;

        while (avail)
        {
            *prxdata++ = hal_usb_read_ep_fifo(&g_usb_handle,(hal_usb_ep_t)ep->address);
            avail--;
        }

        /*MCU has receive all data from OUT FIFO, then clear out packet ready*/
        hal_usb_ep_read_end(&g_usb_handle,(hal_usb_ep_t)ep->address);
    }

    return status;  //return count number
}


void USB_IRQHandler(void)
{
    hal_usb_irq_handler(&g_usb_handle);
}

void USB_ATTACH_IRQHandler(void)
{
    hal_usb_attach_irq_handler(&g_usb_handle);
}

void USB_DETACH_IRQHandler(void)
{
    hal_usb_detach_irq_handler(&g_usb_handle);
}
