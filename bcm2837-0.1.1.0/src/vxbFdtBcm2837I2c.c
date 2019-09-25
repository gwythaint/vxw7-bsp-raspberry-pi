/* vxbFdtBcm2837I2c.c - driver for broadcom 2837 platform I2C module */

/*
 * Copyright (c) 2019 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
modification history
--------------------
08mar19,wdy  created (F11409)
*/

/*
DESCRIPTION
This module implements a driver for the I2C controller presented on broadcom
283X processors. The controller is capable of acting as a master, and each
I2C module instance has one completely independent channel. Access to the I2C
controller is required for certain devices such as EEPROMs and RTCs.

This driver only supports 7 bits addressing, 10 bits addressing is not
supported.

To add the driver to the vxWorks image, add the following component to the
kernel configuration.

\cs
vxprj component add DRV_I2C_FDT_BCM2837
\ce

Each I2c device should be bound to a device tree node which requires
below properties:

\cs
compatible:     Specify the programming model for the device.
                It should be set to "brcm,bcm2837-i2c" and is used
                by vxbus GEN2 for device driver selection.

reg:            Specify the address of the device's resources within
                the address space defined by its parent bus.

def-bus-frequency: Optional, specify the default working clock frequency of the
                I2C BUS in HZ. If not specified, 100000 is used.

interrupts:     Optional, specify interrupt vector of the interrupts that are
                generated by this device, if not specified, poll mode is used.

interrupt-parent: Optional, this property is available to define an interrupt
                parent. if it is missing from a device, it's interrupt parent
                is assumed to be its device tree parent.

clock-frequency: This property is available to define the frequency of
                 clock for system timer module.

sda-pin:         This property is available to define the pin number for SDA.

scl-pin:         This property is available to define the pin number for SCL.

polled-mode:      Specifies work mode. Only 1 and 0 are valid values, 1 means
                  polled-mode, 0 means interrupt mode.
                  If this property is not defined, work mode is interrupt mode.

\ce

Below is an example:

\cs
        i2c1: i2c@3f804000
            {
            compatible = "brcm,bcm2837-i2c";
            reg = <0x0 0x3f804000 0x0 0x1000>,
                  <0x0 0x3f200000 0x0 0x00B0>;
            interrupts = <85>;
            interrupt-parent = <&intc>;
            clock-frequency = <150000000>;
            sda-pin = <2>;
            scl-pin = <3>;
            #address-cells = <1>;
            #size-cells = <0>;

            at24c02:eeprom@50
                {
                compatible = "at24, at24c02";
                reg = <0x50>;
                data-scl-frequency = <100000>;
                };
            };

\ce

INCLUDE FILES: vxBus.h vxbI2cLib.h string.h vxbFdtLib.h
*/

/* includes */

#include <vxWorks.h>
#include <vsbConfig.h>
#include <stdlib.h>
#include <string.h>
#include <semLib.h>
#include <taskLib.h>
#include <stdio.h>
#include <ioLib.h>
#include <hwif/vxBus.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/timer/vxbTimerLib.h>
#include <subsys/clk/vxbClkLib.h>
#include <hwif/buslib/vxbFdtI2cLib.h>

#include "vxbFdtBcm2837I2c.h"

/* defines */

#undef  I2C_DBG_ON
#ifdef  I2C_DBG_ON

#define I2C_DBG_IRQ         0x00000001
#define I2C_DBG_RW          0x00000002
#define I2C_DBG_ERR         0x00000004
#define I2C_DBG_ALL         0xffffffff
#define I2C_DBG_OFF         0x00000000

LOCAL UINT32 i2cDbgMask = I2C_DBG_ALL;
IMPORT FUNCPTR _func_logMsg;

#define I2C_DBG(mask, ...)                              \
    if ((i2cDbgMask & mask) || (mask == I2C_DBG_ALL))   \
        if (_func_logMsg != NULL)                       \
            (* _func_logMsg) (__VA_ARGS__)
#else
#define I2C_DBG(...)
#endif  /* I2C_DBG_ON */

/* forward declarations */

LOCAL STATUS              bcm2837I2cProbe (struct vxbDev * pDev);
LOCAL STATUS              bcm2837I2cAttach (VXB_DEV_ID pDev);
LOCAL VXB_RESOURCE *      bcm2837I2cResAlloc (VXB_DEV_ID pDev,
                                              VXB_DEV_ID pChild,
                                              UINT32 id);
LOCAL STATUS              bcm2837I2cResFree (VXB_DEV_ID pDev,
                                             VXB_DEV_ID pChild,
                                             VXB_RESOURCE * pRes);
LOCAL VXB_RESOURCE_LIST * bcm2837I2cResListGet (VXB_DEV_ID pDev,
                                                VXB_DEV_ID pChild);
LOCAL STATUS              bcm2837I2cTransfer (VXB_DEV_ID pDev,
                                              I2C_MSG *msgs,
                                              int num);
LOCAL VXB_FDT_DEV *       bcm2837I2cDevGet (VXB_DEV_ID pDev,
                                            VXB_DEV_ID pChild);
LOCAL STATUS              bcm2837I2cInit     (BCM_I2C_DRV_CTRL * pDrvCtrl);
LOCAL STATUS              bcm2837I2cPollRdWr (BCM_I2C_DRV_CTRL * pDrvCtrl);
LOCAL void                bcm2837I2cIsr (VXB_DEV_ID pDev);
LOCAL void                bcm2837I2cProbeChild (VXB_DEV_ID Dev,
                                                VXB_FDT_DEV * pParFdtDev);
LOCAL void                bcm2837I2cWrTxFifo (BCM_I2C_DRV_CTRL * pDrvCtrl);
LOCAL void                bcm2837I2cRdRxFifo (BCM_I2C_DRV_CTRL * pDrvCtrl);

LOCAL VXB_DRV_METHOD bcm2837I2cDeviceMethods[] =
    {
    { VXB_DEVMETHOD_CALL (vxbDevProbe),       (FUNCPTR)bcm2837I2cProbe},
    { VXB_DEVMETHOD_CALL (vxbDevAttach),      (FUNCPTR)bcm2837I2cAttach},
    { VXB_DEVMETHOD_CALL (vxbResourceAlloc),  (FUNCPTR)bcm2837I2cResAlloc},
    { VXB_DEVMETHOD_CALL (vxbResourceFree),   (FUNCPTR)bcm2837I2cResFree},
    { VXB_DEVMETHOD_CALL (vxbResourceListGet),(FUNCPTR)bcm2837I2cResListGet},
    { VXB_DEVMETHOD_CALL (vxbI2cXfer),        (FUNCPTR)bcm2837I2cTransfer},
    { VXB_DEVMETHOD_CALL (vxbFdtDevGet),      (FUNCPTR)bcm2837I2cDevGet},
    VXB_DEVMETHOD_END
    };

VXB_DRV vxbOfBcm2837I2cCtlrDrv =
    {
    { NULL } ,
    "bcm2837-i2c",                      /* Name */
    "boardcom 2837 I2C controller",     /* Description */
    VXB_BUSID_FDT,                      /* Class */
    0,                                  /* Flags */
    0,                                  /* Reference count */
    bcm2837I2cDeviceMethods             /* Method table */
    };

LOCAL const VXB_FDT_DEV_MATCH_ENTRY bcm2837I2cMatch[] =
    {
        {
        "brcm,bcm2837-i2c",             /* compatible */
        (void *)NULL
        },
        {}                              /* Empty terminated list */
    };

VXB_DRV_DEF (vxbOfBcm2837I2cCtlrDrv)

/******************************************************************************
*
* bcm2837I2cProbe - probe for device presence at specific address
*
* This routine checks for the I2C controller (or compatible) device at the
* base address. We assume one is present at that address, but we need to verify.
*
* RETURNS: OK if probe passes and assumed a valid I2C controller
* (or compatible) device. ERROR otherwise.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cProbe
    (
    VXB_DEV_ID pDev
    )
    {
    return vxbFdtDevMatch (pDev, bcm2837I2cMatch, NULL);
    }

/*****************************************************************************
*
* bcm2837I2cAttach - VxBus attcach method
*
* This function implements the VxBus attach method for a I2C controller
* device instance.
*
* RETURNS: OK, or ERROR if failed to attach.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cAttach
    (
    VXB_DEV_ID         pDev
    )
    {
    BCM_I2C_DRV_CTRL *   pDrvCtrl = NULL;
    VXB_CLK_ID           pClk = NULL;
    VXB_RESOURCE_ADR *   pResAdr = NULL;
    VXB_RESOURCE *       pRes = NULL;
    VXB_RESOURCE *       pResGpio = NULL;
    VXB_FDT_DEV *        pFdtDev;
    UINT64               i2cInClk;
    void *               pValue;
    UINT32               polled_mode;

    if (pDev == NULL)
        {
        return ERROR;
        }

    pFdtDev = vxbFdtDevGet (pDev);
    if (pFdtDev == NULL)
        {
        return ERROR;
        }

    /* create controller driver context structure for core */

    pDrvCtrl = (BCM_I2C_DRV_CTRL *) vxbMemAlloc (sizeof (BCM_I2C_DRV_CTRL));
    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }

    /* save pDrvCtrl in VXB_DEVICE structure */

    vxbDevSoftcSet (pDev, pDrvCtrl);

    /* save instance ID */

    pDrvCtrl->i2cDev   = pDev;

    if (vxbClkEnableAll (pDev) == ERROR)
        {
        I2C_DBG (I2C_DBG_ERR, "Failed to enable clocks\n",0,0,0,0,0,0);
        return ERROR;
        }

    /* get input clock frequency */

    pClk = vxbClkGet (pDev, NULL);
    if (pClk == NULL)
        {
        I2C_DBG (I2C_DBG_ERR, "Failed to get clock ID.\n",0,0,0,0,0,0);
        (void) vxbClkDisableAll (pDev);
        return ERROR;
        }

    i2cInClk = vxbClkRateGet (pClk);
    if (i2cInClk == CLOCK_RATE_INVALID)
        {
        I2C_DBG (I2C_DBG_ERR,
                   "Failed to get clock rate. Using default frequency\n",
                   0,0,0,0,0,0);
        i2cInClk = BCM2837_I2C_ICLK_FREQ;
        }

    I2C_DBG (I2C_DBG_ERR,
               "I2c clock rate: Using default frequency: %llu \n",
               i2cInClk,0,0,0,0,0);

    pRes = vxbResourceAlloc(pDev, VXB_RES_MEMORY, 0);
    if (pRes == NULL)
        {
        I2C_DBG (I2C_DBG_ERR,
                "am38xxI2cAttach failed to alloc memory resource\n",
                 1, 2, 3, 4, 5, 6);
        goto errOut;
        }

    pResAdr = (VXB_RESOURCE_ADR *)pRes->pRes;
    if (pResAdr == NULL)
        {
        I2C_DBG (I2C_DBG_ERR,
                "bcm2837I2cAttach failed to alloc memory address resource\n",
                 1, 2, 3, 4, 5, 6);
        goto errOut;
        }

    pDrvCtrl->i2cHandle = pResAdr->pHandle;
    pDrvCtrl->i2cRegBase = pResAdr->virtual;

    pResGpio = vxbResourceAlloc(pDev, VXB_RES_MEMORY, 1);
    if (pResGpio == NULL)
        {
        I2C_DBG (I2C_DBG_ERR,
                "am38xxI2cAttach failed to alloc memory resource\n",
                 1, 2, 3, 4, 5, 6);
        goto errOut;
        }

    pResAdr = (VXB_RESOURCE_ADR *)pResGpio->pRes;
    if (pResAdr == NULL)
        {
        I2C_DBG (I2C_DBG_ERR,
                "bcm2837I2cAttach failed to alloc memory address resource\n",
                 1, 2, 3, 4, 5, 6);
        goto errOut;
        }

    pDrvCtrl->pGpioHandle = pResAdr->pHandle;
    pDrvCtrl->gpioRegBase = pResAdr->virtual;

    if ((pValue = (void*)vxFdtPropGet (pFdtDev->offset,
                                       I2C_BUS_SPEED_CFG_NAME, NULL)) != NULL)
        {
        pDrvCtrl->busSpeed = vxFdt32ToCpu (*(UINT32 *) pValue);
        }
    else
        {
        pDrvCtrl->busSpeed = STD_MODE;
        }
    pDrvCtrl->defBusSpeed = pDrvCtrl->busSpeed;

    pDrvCtrl->iClkFreq = i2cInClk;

    if ((pValue = (void*)vxFdtPropGet (pFdtDev->offset,
                             BCM2837_GPIO_PROPERTY_FOR_I2C_SDA, NULL)) != NULL)
        {
        pDrvCtrl->i2cSdaPin = vxFdt32ToCpu (*(UINT32 *) pValue);
        }
    else
        {
         goto errOut;
        }

    if ((pValue = (void *) vxFdtPropGet (pFdtDev->offset,
                             BCM2837_GPIO_PROPERTY_FOR_I2C_SCL, NULL)) != NULL)
        {
        pDrvCtrl->i2cSclPin = vxFdt32ToCpu (*(UINT32 *) pValue);
        }
    else
        {
         goto errOut;
        }

    /* reset I2C module */

    if (bcm2837I2cInit (pDrvCtrl) == ERROR)
        goto errOut;

    pDrvCtrl->dataBuf = NULL;
    pDrvCtrl->i2cDevSem = semMCreate (SEM_Q_PRIORITY | SEM_DELETE_SAFE |
                                      SEM_INVERSION_SAFE);
    if (pDrvCtrl->i2cDevSem == SEM_ID_NULL)
        {
        I2C_DBG (I2C_DBG_ERR,
                 "bcm2837I2cAttach: i2cDevSem create failed\n",
                 0, 0, 0, 0, 0, 0);
        goto errOut;
        }

    pDrvCtrl->i2cDataSem = semBCreate (SEM_Q_FIFO, SEM_EMPTY);
    if (pDrvCtrl->i2cDataSem == SEM_ID_NULL)
        {
        I2C_DBG (I2C_DBG_ERR,
                 "bcm2837I2cAttach: i2cDataSem Ceate failed\n",
                 0, 0, 0, 0, 0, 0);
        goto errOut;
        }

    pValue = (void *) vxFdtPropGet (pFdtDev->offset, "polled-mode", NULL);
    if (pValue != NULL)
        {
        polled_mode = vxFdt32ToCpu (*(UINT32 *) pValue);
        if ((1 != polled_mode) && (0 != polled_mode))
            {
            I2C_DBG (I2C_DBG_ERR,
                 "bcm2837I2cAttach: wrong configuration on polled-mode %d\n",
                 polled_mode, 0, 0, 0, 0, 0);
            goto errOut;
            }
        else
            {
            pDrvCtrl->polling = (BOOL) polled_mode;
            }
        }
    else
        {
        pDrvCtrl->polling = FALSE;
        }

    if (!pDrvCtrl->polling)
        {

        /* allocate interrupt resource, connect and enable interrupt */

        pDrvCtrl->intRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);
        if (pDrvCtrl->intRes == NULL)
            {
            I2C_DBG (I2C_DBG_ERR,
                 "bcm2837I2cAttach: vxbResourceAlloc VXB_RES_IRQ fail.\n",
                 0, 0, 0, 0, 0, 0);
            goto errOut;
            }
        else
            {
            if (OK != vxbIntConnect (pDev, pDrvCtrl->intRes,
                                     (VOIDFUNCPTR) bcm2837I2cIsr,
                                     pDev))
                {
                 I2C_DBG (I2C_DBG_ERR,
                 "bcm2837I2cAttach: vxbIntConnect fail.\n",
                 0, 0, 0, 0, 0, 0);
                 goto errOut;
                }
            else
                {
                if (OK != vxbIntEnable (pDev, pDrvCtrl->intRes))
                    {
                    I2C_DBG (I2C_DBG_ERR,
                             "bcm2837I2cAttach: vxbIntEnable fail.\n",
                              0, 0, 0, 0, 0, 0);
                    (void) vxbIntDisconnect (pDev, pDrvCtrl->intRes);
                    goto errOut;
                    }
                }
            }
        }

    bcm2837I2cProbeChild (pDev, pFdtDev);

    return OK;

errOut:
    if (pDrvCtrl->i2cDataSem != NULL)
        (void)semDelete (pDrvCtrl->i2cDataSem);
    if (pDrvCtrl->i2cDevSem != NULL)
        (void)semDelete (pDrvCtrl->i2cDevSem);
    if (pRes != NULL)
        (void)vxbResourceFree (pDev, pRes);
    if (pResGpio != NULL)
        (void)vxbResourceFree (pDev, pResGpio);
    if (pDrvCtrl->intRes != NULL)
        (void)vxbResourceFree (pDev, pDrvCtrl->intRes);

    vxbMemFree (pDrvCtrl);
    vxbDevSoftcSet (pDev, NULL);

    return ERROR;
    }

/*******************************************************************************
*
* bcm2837I2cDevGet - get the FDT child device information
*
* This routine gets the FDT child device information
*
* RETURNS: the device information pointer or NULL if failed to get.
*
* ERRNO: N/A
*/

LOCAL VXB_FDT_DEV * bcm2837I2cDevGet
    (
    VXB_DEV_ID pDev,
    VXB_DEV_ID pChild
    )
    {
    I2C_DEV_INFO * pBcm2837I2cDevInfo;

    if (pChild == NULL)
        return NULL;

    pBcm2837I2cDevInfo = (I2C_DEV_INFO *)vxbDevIvarsGet(pChild);

    if (pBcm2837I2cDevInfo == NULL)
        return NULL;

    return &pBcm2837I2cDevInfo->vxbFdtDev;
    }

/*****************************************************************************
*
* bcm2837I2cSetDivider - set the divider to get the bus speed
*
* This routine initializes the divider to get the bus speed.
*
* RETURNS: OK, or ERROR if failed to initialize.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cSetDivider
    (
    BCM_I2C_DRV_CTRL * pI2c
    )
    {
    UINT32 divider;
    UINT32 redl;
    UINT32 fedl;

    divider = (UINT32) BCM2837_DIV_ROUND_UP (pI2c->iClkFreq, pI2c->busSpeed);

    /*
     * the register is always interpreted as an even number,
     * by rounding down. In other words, the LSB is ignored. So,
     * if the LSB is set, increment the divider to avoid any issue.
     */

    if ((divider & 1) != 0)
        {
        divider++;
        }
    if ((divider < BCM2837_I2C_CDIV_MIN) ||
        (divider > BCM2837_I2C_CDIV_MAX))
        {
        I2C_DBG (I2C_DBG_ERR,
                "bcm2837I2cSetDivider:I2C divider set error\n",
                 0, 0, 0, 0, 0, 0);
        return ERROR;
        }

    BCM_I2C_REG_WRITE (pI2c, BCM2837_I2C_CLKDIV_REG, divider);

    /*
     * Number of core clocks to wait after falling edge before
     * outputting the next data bit.  Note that both FEDL and REDL
     * can't be greater than CDIV/2.
     */

    fedl = max(divider / 16, 1u);

    /*
     * Number of core clocks to wait after rising edge before
     * sampling the next incoming data bit.
     */

    redl = max(divider / 4, 1u);

    BCM_I2C_REG_WRITE (pI2c, BCM2837_I2C_DATADLY_REG,
                      (fedl << BCM2837_I2C_FEDL_SHIFT) |
                      (redl << BCM2837_I2C_REDL_SHIFT));
    return OK;
    }

/*****************************************************************************
*
* bcm2837I2cInit - initialize the I2C module
*
* This routine initializes the I2C module.
*
* RETURNS: OK, or ERROR if failed to initialize.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cInit
    (
    BCM_I2C_DRV_CTRL * pDrvCtrl
    )
    {
    UINT32 val = 0;

    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }

    /*
     * initialize gpio for i2c alterative function. Users must
     * confirm that pins are properly configured because they
     * do not support mutually exclusive with GPIO driver.
     */

    val = BCM_I2C_GPIO_REG_READ (pDrvCtrl,
                                 BCM2837_GPIO_FSEL_REG (pDrvCtrl->i2cSdaPin));

    if (((val >> BCM2837_GPIO_FSEL_SHIFT (pDrvCtrl->i2cSdaPin)) &
         BCM2837_GPIO_FSEL_MASK) != BCM2837_GPIO_FSEL_ALT0)
        {
        val &= ~ (BCM2837_GPIO_FSEL_MASK <<
                  BCM2837_GPIO_FSEL_SHIFT (pDrvCtrl->i2cSdaPin));
        val |= BCM2837_GPIO_FSEL_ALT0 <<
               BCM2837_GPIO_FSEL_SHIFT (pDrvCtrl->i2cSdaPin);

        BCM_I2C_GPIO_REG_WRITE (pDrvCtrl,
                                BCM2837_GPIO_FSEL_REG (pDrvCtrl->i2cSdaPin),
                                val);
        }

    val = BCM_I2C_GPIO_REG_READ (pDrvCtrl,
                                 BCM2837_GPIO_FSEL_REG ( pDrvCtrl->i2cSclPin));
    if (((val >> BCM2837_GPIO_FSEL_SHIFT (pDrvCtrl->i2cSclPin)) &
         BCM2837_GPIO_FSEL_MASK) != BCM2837_GPIO_FSEL_ALT0)
        {
        val &= ~ (BCM2837_GPIO_FSEL_MASK <<
                  BCM2837_GPIO_FSEL_SHIFT (pDrvCtrl->i2cSclPin));
        val |= BCM2837_GPIO_FSEL_ALT0 <<
               BCM2837_GPIO_FSEL_SHIFT (pDrvCtrl->i2cSclPin);

        BCM_I2C_GPIO_REG_WRITE (pDrvCtrl,
                                BCM2837_GPIO_FSEL_REG (pDrvCtrl->i2cSclPin),
                                val);
        }

    (void) bcm2837I2cSetDivider (pDrvCtrl);

    BCM_I2C_REG_CLEARBIT_4 (pDrvCtrl, BCM2837_I2C_CONTROL_REG,
                            BCM2837_I2C_C_INTD |
                            BCM2837_I2C_C_INTR |
                            BCM2837_I2C_C_INTT |
                            BCM2837_I2C_C_I2CEN);
    return OK;
    }

/*****************************************************************************
*
* bcm2837I2cStartTransfer - trigger start sending condition for I2C
*
* This routine triggers start sending condition for I2C
*
* RETURNS: OK, or ERROR if failed to trigger.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cStartTransfer
    (
    BCM_I2C_DRV_CTRL * pDrvCtrl
    )
    {
    UINT32 mControlReg = BCM2837_I2C_C_I2CEN | BCM2837_I2C_C_ST;
    BOOL   mIsLastMsg  = (pDrvCtrl->numOfMsgs == 1);

    if (pDrvCtrl->numOfMsgs == 0)
        {
        return ERROR;
        }

    pDrvCtrl->numOfMsgs--;
    pDrvCtrl->dataBuf = pDrvCtrl->msgs->buf;
    pDrvCtrl->dataLength = pDrvCtrl->msgs->len;

    if ((pDrvCtrl->msgs->flags & I2C_M_RD) == I2C_M_RD)
        {
        mControlReg |= BCM2837_I2C_C_READ;
        }

    if ((pDrvCtrl->intEnabled) && (!pDrvCtrl->polling))
        {
        if ((pDrvCtrl->msgs->flags & I2C_M_RD) == I2C_M_RD)
            {
            mControlReg |= BCM2837_I2C_C_INTR;
            }
        else
            {
            mControlReg |= BCM2837_I2C_C_INTT;
            }

        if (mIsLastMsg)
            {
            mControlReg |= BCM2837_I2C_C_INTD;
            }
        }

    BCM_I2C_REG_WRITE (pDrvCtrl, BCM2837_I2C_SADDR_REG, pDrvCtrl->dataSAddr);
    BCM_I2C_REG_WRITE (pDrvCtrl, BCM2837_I2C_DLEN_REG, pDrvCtrl->dataLength);
    BCM_I2C_REG_WRITE (pDrvCtrl, BCM2837_I2C_CONTROL_REG, mControlReg);

    return OK;
    }

/******************************************************************************
*
* bcm2837I2cWrTxFifo - fill the TX FIFO
*
* This routine checks the TXD flag in status register, and fill data into
* the FIFO if TXD flag was set by hardware.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837I2cWrTxFifo
    (
    BCM_I2C_DRV_CTRL * pDrvCtrl
    )
    {
    UINT32 val;

    while(pDrvCtrl->dataLength != 0)
        {
        val = BCM_I2C_REG_READ (pDrvCtrl, BCM2837_I2C_STATUS_REG);
        if ((val & BCM2837_I2C_STATUS_TXD) == 0)
            {
            break;
            }

        BCM_I2C_REG_WRITE (pDrvCtrl,
                           BCM2837_I2C_DFIFO_REG,
                           *pDrvCtrl->dataBuf);

        pDrvCtrl->dataBuf++;
        pDrvCtrl->dataLength --;
        }
    }

/******************************************************************************
*
* bcm2837I2cRdRxFifo - read the data from FIFO
*
* This routine checks RXD flag in status register,and read data from FIFO if
* the RXD flag was set by hardware.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837I2cRdRxFifo
    (
    BCM_I2C_DRV_CTRL * pDrvCtrl
    )
    {
    UINT32 val;

    while(pDrvCtrl->dataLength != 0)
        {
        val = BCM_I2C_REG_READ (pDrvCtrl, BCM2837_I2C_STATUS_REG);
        if ((val & BCM2837_I2C_STATUS_RXD) == 0)
            {
            break;
            }

        *pDrvCtrl->dataBuf = (UINT8) BCM_I2C_REG_READ (pDrvCtrl,
                                                       BCM2837_I2C_DFIFO_REG);
        pDrvCtrl->dataBuf++;
        pDrvCtrl->dataLength--;
        }
    }

/*******************************************************************************
*
* bcm2837I2cIsr - interrupt service routine
*
* This routine handles interrupts of I2C.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837I2cIsr
    (
    VXB_DEV_ID      pDev
    )
    {
    UINT32               status;
    BCM_I2C_DRV_CTRL *   pDrvCtrl;
    UINT32               err;

    if (pDev == NULL)
        {
        return;
        }

    pDrvCtrl = (BCM_I2C_DRV_CTRL*) vxbDevSoftcGet (pDev);
    if (pDrvCtrl == NULL)
        {
        return;
        }

    status = BCM_I2C_REG_READ (pDrvCtrl, BCM2837_I2C_STATUS_REG);
    err = status & (BCM2837_I2C_STATUS_CLKT | BCM2837_I2C_STATUS_ERR);

    if (err != 0)
        {
        pDrvCtrl->msgError = err;
        goto endTransfer;
        }

    if ((status & BCM2837_I2C_STATUS_DONE) != 0)
        {
        if (pDrvCtrl->msgs == NULL)
            {
            I2C_DBG (I2C_DBG_IRQ,
                     "bcm2837I2cIsr: unknown interrupt.\n", 0, 0, 0, 0, 0, 0);
            return;
            }
        if ((pDrvCtrl->msgs->flags & I2C_M_RD) == I2C_M_RD)
            {
            bcm2837I2cRdRxFifo (pDrvCtrl);
            status = BCM_I2C_REG_READ (pDrvCtrl, BCM2837_I2C_STATUS_REG);
            }

        if (((status & BCM2837_I2C_STATUS_RXD) != 0) ||
            (pDrvCtrl->dataLength != 0))
            {
            pDrvCtrl->msgError = BCM2837_I2C_STATUS_LEN;
            }
        else
            {
            pDrvCtrl->msgError = OK;
            }

        goto endTransfer;

        }

    if ((status & BCM2837_I2C_STATUS_TXW) != 0)
        {

        /* transmit buffer has NULL */

        if (pDrvCtrl->dataLength == 0)
            {
            pDrvCtrl->msgError |= BCM2837_I2C_STATUS_LEN;
            goto endTransfer;
            }

        bcm2837I2cWrTxFifo (pDrvCtrl);

        if ((pDrvCtrl->numOfMsgs != 0) &&
            (pDrvCtrl->dataLength == 0))
            {
            pDrvCtrl->msgs++;
            if (bcm2837I2cStartTransfer (pDrvCtrl) != OK)
                {
                return;
                }
            }
        return;
        }

    if ((status & BCM2837_I2C_STATUS_RXR) != 0)
        {
        if (pDrvCtrl->dataLength == 0)
            {
            pDrvCtrl->msgError |= BCM2837_I2C_STATUS_LEN;
            goto endTransfer;
            }
        bcm2837I2cRdRxFifo (pDrvCtrl);
        return;
        }
    return;

endTransfer:
    BCM_I2C_REG_WRITE(pDrvCtrl, BCM2837_I2C_CONTROL_REG, BCM2837_I2C_C_CLEAR);
    BCM_I2C_REG_WRITE(pDrvCtrl, BCM2837_I2C_STATUS_REG,
                                BCM2837_I2C_STATUS_CLKT |
                                BCM2837_I2C_STATUS_ERR |
                                BCM2837_I2C_STATUS_DONE);

    (void) semGive(pDrvCtrl->i2cDataSem);
    return;
    }

/*****************************************************************************
*
* bcm2837I2cProbeChild - probe child device
*
* This function probes child devices.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void bcm2837I2cProbeChild
    (
    VXB_DEV_ID    pDev,
    VXB_FDT_DEV * pParFdtDev
    )
    {
    int offset = pParFdtDev->offset;
    VXB_DEV_ID pCur = NULL;
    I2C_DEV_INFO *pI2cDevInfo;
    VXB_FDT_DEV * pFdtDev;

    for (offset = VX_FDT_CHILD(offset); offset > 0;
         offset = VX_FDT_PEER(offset))
        {
        pCur = NULL;

        if (vxbDevCreate (&pCur) != OK)
            continue;

        pI2cDevInfo = (I2C_DEV_INFO *)vxbMemAlloc(sizeof(*pI2cDevInfo));

        if (pI2cDevInfo == NULL)
            {
            (void) vxbDevDestroy(pCur);
            continue;
            }

        pFdtDev = &pI2cDevInfo->vxbFdtDev;

        vxbFdtDevSetup(offset, pFdtDev);
        vxbDevNameSet(pCur, pFdtDev->name, FALSE);

        /* Get the device register and interrupt infomation  */

        if (vxbResourceInit(&pI2cDevInfo->vxbResList) != OK)
            {
            (void) vxbDevDestroy (pCur);
            vxbMemFree (pI2cDevInfo);
            continue;
            }

        if(vxbFdtRegGet(&pI2cDevInfo->vxbResList, pFdtDev) == ERROR)
            {
            vxbFdtResFree(&pI2cDevInfo->vxbResList);
            vxbMemFree(pI2cDevInfo);
            (void) vxbDevDestroy(pCur);
            continue;
            }

        if (vxbFdtIntGet(&pI2cDevInfo->vxbResList, pFdtDev) == ERROR)
            {
            vxbFdtResFree(&pI2cDevInfo->vxbResList);
            vxbMemFree(pI2cDevInfo);
            (void) vxbDevDestroy(pCur);
            continue;
            }

        /* Assign the bus internal variable and type  */

        vxbDevIvarsSet(pCur, (void *) pI2cDevInfo);
        vxbDevClassSet(pCur, VXB_BUSID_FDT);

        (void) vxbDevAdd(pDev, pCur);
        }
    }

/*******************************************************************************
*
* bcm2837I2cResAlloc - allocate resource for child device
*
* This routine allocates resource for child device and go corresponding
* operation based on its type.
*
* RETURNS: allocated resource pointer, or NULL if failure.
*
* ERRNO: N/A
*/

LOCAL VXB_RESOURCE * bcm2837I2cResAlloc
    (
    VXB_DEV_ID pDev,
    VXB_DEV_ID pChild,
    UINT32 id
    )
    {
    I2C_DEV_INFO *pBcmDevInfo;
    VXB_RESOURCE * vxbRes;
    VXB_RESOURCE_ADR * vxbAdrRes;

    pBcmDevInfo= (I2C_DEV_INFO *)vxbDevIvarsGet(pChild);

    vxbRes = vxbResourceFind(&pBcmDevInfo->vxbResList, id);

    if (vxbRes == NULL)
        return NULL;

    if ((VXB_RES_TYPE(vxbRes->id) == VXB_RES_MEMORY) ||
         (VXB_RES_TYPE(vxbRes->id) == VXB_RES_IO))
        {
        vxbAdrRes = vxbRes->pRes;
        vxbAdrRes->virtual = (VIRT_ADDR)(vxbAdrRes->start);
        return vxbRes;
        }
    else if ((VXB_RES_TYPE(vxbRes->id) == VXB_RES_IRQ) &&
             (vxbIntMap (vxbRes) == OK))
        return vxbRes;
    else
        {
        return NULL;
        }
    }

/*******************************************************************************
*
* bcm2837I2cResFree - free the resource for child device
*
* This routine frees the resource allocated by calling am38xxI2cResAlloc.
*
* RETURNS: OK if free successfully, otherwise ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cResFree
    (
    VXB_DEV_ID pDev,
    VXB_DEV_ID pChild,
    VXB_RESOURCE * pRes
    )
    {
    I2C_DEV_INFO * pI2cDevInfo;

    pI2cDevInfo = (I2C_DEV_INFO *) vxbDevIvarsGet (pChild);

    if ((pChild == NULL) || (pRes == NULL))
        return ERROR;

    return vxbResourceRemove (&pI2cDevInfo->vxbResList, pRes);
    }

/*******************************************************************************
*
* bcm2837I2cResListGet - get the resource list of specific device
*
* This routine gets the resource list of specific device.
*
* RETURNS: resource list pointer, or NULL if failed to get.
*
* ERRNO: N/A
*/

LOCAL VXB_RESOURCE_LIST * bcm2837I2cResListGet
    (
    VXB_DEV_ID pDev,
    VXB_DEV_ID pChild
    )
    {
    I2C_DEV_INFO * pI2cDevInfo;

    pI2cDevInfo = (I2C_DEV_INFO *)vxbDevIvarsGet (pChild);

    if (pI2cDevInfo == NULL)
        return NULL;

    return  &pI2cDevInfo->vxbResList;
    }

/*****************************************************************************
*
* bcm2837I2cPollRdWr - read or write a sequence of bytes from or to a device
*
* This function reads <length> bytes from the I2C device and stores them
* in the buffer specified by <pDataBuf> or writes <length> bytes to the I2C
* device read from the buffer specified by <pDataBuf>. They begin at an offset
* specified previously in the transaction. or  If too many bytes are requested,
* the read will return ERROR, though any successfully read bytes will be
* returned in <pDataBuf>.
*
* RETURNS: OK if read completed successfully, otherwise ERROR
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cPollRdWr
    (
    BCM_I2C_DRV_CTRL * pDrvCtrl
    )
    {
    UINT32 status;

    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }

    if (pDrvCtrl->msgs == NULL)
        {
        return ERROR;
        }

    if (bcm2837I2cStartTransfer (pDrvCtrl) != OK)
        {
        return ERROR;
        }

    status = BCM_I2C_REG_READ (pDrvCtrl, BCM2837_I2C_STATUS_REG);
    I2C_DBG (I2C_DBG_ERR, "bcm2837I2cPollRead:status-0x%08x.\n",
             status, 0, 0, 0, 0, 0);
    while (1)
        {
        status = BCM_I2C_REG_READ (pDrvCtrl, BCM2837_I2C_STATUS_REG);
        if ((status & (BCM2837_I2C_STATUS_CLKT | BCM2837_I2C_STATUS_ERR)) != 0)
            {
            break;
            }

        if ((status & BCM2837_I2C_STATUS_DONE) != 0)
            {
            if (pDrvCtrl->msgs->flags & I2C_M_RD)
                {
                bcm2837I2cRdRxFifo (pDrvCtrl);
                }
            break;
            }

        if (status & BCM2837_I2C_STATUS_RXR)
            {
            if (pDrvCtrl->dataLength != 0)
                {
                bcm2837I2cRdRxFifo (pDrvCtrl);
                }
            else
                {
                pDrvCtrl->msgError = BCM2837_I2C_STATUS_LEN;
                break;
                }
            }
        if ((status & BCM2837_I2C_STATUS_TXW) != 0)
            {
            if (pDrvCtrl->dataLength != 0)
                {
                bcm2837I2cWrTxFifo (pDrvCtrl);
                if ((pDrvCtrl->numOfMsgs != 0) &&
                    (pDrvCtrl->dataLength == 0))
                    {
                    pDrvCtrl->msgs++;
                    if (bcm2837I2cStartTransfer (pDrvCtrl) != OK)
                        {
                        return ERROR;
                        }
                    }
                }
                else
                    {
                    pDrvCtrl->msgError = BCM2837_I2C_STATUS_LEN;
                    break;
                    }
            }
        }

    BCM_I2C_REG_WRITE (pDrvCtrl,
                       BCM2837_I2C_CONTROL_REG,
                       BCM2837_I2C_C_CLEAR);
    BCM_I2C_REG_WRITE (pDrvCtrl,
                       BCM2837_I2C_STATUS_REG,
                       BCM2837_I2C_STATUS_CLKT |
                       BCM2837_I2C_STATUS_ERR |
                       BCM2837_I2C_STATUS_DONE);

    if (pDrvCtrl->msgs->wrTime != 0)
        {
        vxbUsDelay((int) pDrvCtrl->msgs->wrTime);
        }

    return OK;
    }

/*******************************************************************************
*
* bcm2837I2cTransfer - VxBus I2C device generic write support routine
*
* This function supplies the whole I2C transfer operations and be called by
* vxbI2cDevXfer. It transfers num messages on a given I2C target.
*
* RETURNS: OK, or ERROR if the transfer is not complete successfully.
*
* ERRNO: N/A
*/

LOCAL STATUS bcm2837I2cTransfer
    (
    VXB_DEV_ID      pDev,
    I2C_MSG *       msgs,
    int             num
    )
    {
    BCM_I2C_DRV_CTRL *  pDrvCtrl;
    I2C_MSG *           pmsgs = msgs;
    int                 i;

    if (pDev == NULL || msgs == NULL || num <= 0)
        return ERROR;

    pDrvCtrl = (BCM_I2C_DRV_CTRL *) vxbDevSoftcGet (pDev);
    if (pDrvCtrl == NULL)
        return ERROR;

    pDrvCtrl->intEnabled = sysIntEnableFlagGet ();
    if (pDrvCtrl->intEnabled)
        {
        if (semTake(pDrvCtrl->i2cDevSem, WAIT_FOREVER) == ERROR)
            {
            I2C_DBG (I2C_DBG_ERR,
                     "bcm2837I2cTransfer:semTake error\n", 0, 0, 0, 0, 0, 0);
            return ERROR;
            }
        }

    pDrvCtrl->msgs = msgs;
    pDrvCtrl->numOfMsgs = num;
    pDrvCtrl->msgError = 0;

    if (msgs->scl == 0)
        {
        msgs->scl = pDrvCtrl->defBusSpeed;
        }
    if (msgs->scl != pDrvCtrl->busSpeed)
        {
        pDrvCtrl->busSpeed = msgs->scl;
        if (bcm2837I2cSetDivider(pDrvCtrl) != OK)
            {

            if (pDrvCtrl->intEnabled)
                {
                (void) semGive(pDrvCtrl->i2cDevSem);
                }

            return ERROR;
            }
        }

    pDrvCtrl->dataSAddr = msgs->addr;
    pDrvCtrl->totalLen = 0;

    for (i = 0; i < num; i++)
        {
        pDrvCtrl->totalLen += pmsgs->len;
        pmsgs++;
        }

    if ((pDrvCtrl->intEnabled) && (! pDrvCtrl->polling))
        {
        if (bcm2837I2cStartTransfer(pDrvCtrl) != OK)
            {
            if (pDrvCtrl->intEnabled)
                {
                (void) semGive(pDrvCtrl->i2cDevSem);
                }
            return ERROR;
            }

        if (semTake(pDrvCtrl->i2cDataSem, SEM_TIMEOUT * pDrvCtrl->totalLen)
                   == ERROR)
            {
             I2C_DBG (I2C_DBG_ERR,
                      "bcm2837I2cTransfer:semTake  i2cDataSem error\n",
                      0, 0, 0, 0, 0, 0);
             (void) semGive(pDrvCtrl->i2cDevSem);
             return ERROR;
            }
        if (pDrvCtrl->msgs->wrTime != 0)
            {
            vxbUsDelay((int) pDrvCtrl->msgs->wrTime);
            }
        }
    else
        {
        if (bcm2837I2cPollRdWr (pDrvCtrl) != OK)
            {
            I2C_DBG (I2C_DBG_ERR,
                      "bcm2837I2cTransfer:poll read and write error\n",
                      0, 0, 0, 0, 0, 0);
             if (pDrvCtrl->intEnabled)
                {
                (void) semGive(pDrvCtrl->i2cDevSem);
                }
            return ERROR;
            }
        }

    if (pDrvCtrl->intEnabled)
        {
        (void) semGive(pDrvCtrl->i2cDevSem);
        }

    if (pDrvCtrl->msgError != 0)
        {
        return ERROR;
        }

    return OK;
}