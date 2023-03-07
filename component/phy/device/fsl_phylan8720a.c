/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_phylan8720a.h"
#include "fsl_enet.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Defines the timeout macro. */
#define PHY_TIMEOUT_COUNT 500000

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

const phy_operations_t phylan8720a_ops = {.phyInit            = PHY_LAN8720A_Init,
                                          .phyWrite           = PHY_LAN8720A_Write,
                                          .phyRead            = PHY_LAN8720A_Read,
										  .getAutoNegoStatus  = PHY_LAN8720A_GetAutoNegotiationStatus,
                                          .getLinkStatus      = PHY_LAN8720A_GetLinkStatus,
                                          .getLinkSpeedDuplex = PHY_LAN8720A_GetLinkSpeedDuplex,
										  .setLinkSpeedDuplex = PHY_LAN8720A_SetLinkSpeedDuplex,
                                          .enableLoopback     = NULL};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t PHY_LAN8720A_GetAutoNegotiationStatus(phy_handle_t *handle, bool *status)
{
    assert(status);

    status_t result;
    uint32_t regValue;

    *status = false;

    /* Check auto negotiation complete. */
    result = MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_BASICSTATUS_REG, &regValue);
    if (result == kStatus_Success)
    {
        if ((regValue & PHY_BSTATUS_AUTONEGCOMP_MASK) != 0U)
        {
            *status = true;
        }
    }
    return result;
}
										  
status_t PHY_LAN8720A_SetLinkSpeedDuplex(phy_handle_t *handle, phy_speed_t speed, phy_duplex_t duplex)
{
    /* This PHY only supports 10/100M speed. */
    assert(speed <= kPHY_Speed100M);

    status_t result;
    uint32_t regValue;

    result = MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_BASICCONTROL_REG, &regValue);
    if (result == kStatus_Success)
    {
        /* Disable the auto-negotiation and set according to user-defined configuration. */
        regValue &= ~PHY_BCTL_AUTONEG_MASK;
        if (speed == kPHY_Speed100M)
        {
            regValue |= PHY_BCTL_SPEED0_MASK;
        }
        else
        {
            regValue &= ~PHY_BCTL_SPEED0_MASK;
        }
        if (duplex == kPHY_FullDuplex)
        {
            regValue |= PHY_BCTL_DUPLEX_MASK;
        }
        else
        {
            regValue &= ~PHY_BCTL_DUPLEX_MASK;
        }
        result = MDIO_Write(handle->mdioHandle, handle->phyAddr, PHY_BASICCONTROL_REG, regValue);
    }
    return result;
}

//status_t PHY_LAN8720A_Init(ENET_Type *base, uint32_t phyAddr, uint32_t srcClock_Hz)
//{
//    uint32_t bssReg;
//    uint32_t counter = PHY_TIMEOUT_COUNT;
//    uint32_t idReg = 0;
//    status_t result = kStatus_Success;
//    uint32_t instance = ENET_GetInstance(base);
//    uint32_t timeDelay;

//#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
//    /* Set SMI first. */
//    CLOCK_EnableClock(s_enetClock[instance]);
//#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
//    ENET_SetSMI(base, srcClock_Hz, false);

//    /* Initialization after PHY stars to work. */
//    while ((idReg != PHY_CONTROL_ID1) && (counter != 0))
//    {
//        PHY_Read(base, phyAddr, PHY_ID1_REG, &idReg);
//        counter --;       
//    }

//    if (!counter)
//    {
//        return kStatus_Fail;
//    }

//    /* Reset PHY. */
//    counter = PHY_TIMEOUT_COUNT;
//    result = PHY_Write(base, phyAddr, PHY_BASICCONTROL_REG, PHY_BCTL_RESET_MASK);
//    if (result == kStatus_Success)
//    {  
//        
//        /* Set the negotiation. */
//        result = PHY_Write(base, phyAddr, PHY_AUTONEG_ADVERTISE_REG,
//                           (PHY_100BASETX_FULLDUPLEX_MASK | PHY_100BASETX_HALFDUPLEX_MASK |
//                            PHY_10BASETX_FULLDUPLEX_MASK | PHY_10BASETX_HALFDUPLEX_MASK | 0x1U));
//        if (result == kStatus_Success)
//        {
//            result = PHY_Write(base, phyAddr, PHY_BASICCONTROL_REG,
//                               (PHY_BCTL_AUTONEG_MASK | PHY_BCTL_RESTART_AUTONEG_MASK));
//            if (result == kStatus_Success)
//            {
//                /* Check auto negotiation complete. */
//                while (counter --)
//                {
//                    result = PHY_Read(base, phyAddr, PHY_BASICSTATUS_REG, &bssReg);
//                    if ( result == kStatus_Success)
//                    {
//                        if ((bssReg & PHY_BSTATUS_AUTONEGCOMP_MASK) != 0)
//                        {
//                            /* Wait a moment for Phy status stable. */
//                            for (timeDelay = 0; timeDelay < PHY_TIMEOUT_COUNT; timeDelay ++)
//                            {
//                                __ASM("nop");
//                            }
//                            break;
//                        }
//                    }

//                    if (!counter)
//                    {
//                        return kStatus_PHY_AutoNegotiateFail;
//                    }
//                }
//            }
//        }
//    }
//    return result;
//}

status_t PHY_LAN8720A_Init(phy_handle_t *handle, const phy_config_t *config)
{
    uint32_t reg = 0;
    uint32_t idReg = 0;
    uint32_t delay = PHY_TIMEOUT_COUNT;
    bool status    = false;
	uint32_t timeDelay;
	status_t result = kStatus_Success;
    /* Init MDIO interface. */
    MDIO_Init(handle->mdioHandle);

    /* assign phy address. */
    handle->phyAddr = config->phyAddr;

    /* Initialization after PHY stars to work. */
    while ((idReg != PHY_CONTROL_ID1) && (delay != 0))
    {
        result = MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_ID1_REG, &idReg);
        delay--;
    }

    if (!delay)
    {
        return kStatus_Fail;
    }
	
    /* Reset PHY. */
    delay = PHY_TIMEOUT_COUNT;
    result = MDIO_Write(handle->mdioHandle, handle->phyAddr, PHY_BASICCONTROL_REG, PHY_BCTL_RESET_MASK);
    if (result == kStatus_Success)
    {  
        /* Set the negotiation. */
        result = MDIO_Write(handle->mdioHandle, handle->phyAddr, PHY_AUTONEG_ADVERTISE_REG,
                           (PHY_100BASETX_FULLDUPLEX_MASK | PHY_100BASETX_HALFDUPLEX_MASK |
                            PHY_10BASETX_FULLDUPLEX_MASK | PHY_10BASETX_HALFDUPLEX_MASK | 0x1U));
        if (result == kStatus_Success)
        {
            result = MDIO_Write(handle->mdioHandle, handle->phyAddr, PHY_BASICCONTROL_REG,
                               (PHY_BCTL_AUTONEG_MASK | PHY_BCTL_RESTART_AUTONEG_MASK));
            if (result == kStatus_Success)
            {
                /* Check auto negotiation complete. */
                while (delay --)
                {
                    result = MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_BASICSTATUS_REG, &reg);
                    if ( result == kStatus_Success)
                    {
                        if (((reg & PHY_BSTATUS_AUTONEGCOMP_MASK) != 0) && (reg & PHY_BSTATUS_LINKSTATUS_MASK))
                        {
                            /* Wait a moment for Phy status stable. */
                            for (timeDelay = 0; timeDelay < PHY_TIMEOUT_COUNT; timeDelay ++)
                            {
                                __ASM("nop");
                            }
                            break;
                        }
                    }
                    if (!delay)
                    {
                        return kStatus_Fail;
                    }
                }
            }
        }
    }
	
//    delay = PHY_TIMEOUT_COUNT;

//    /* Reset PHY and wait until completion. */
//    MDIO_Write(handle->mdioHandle, handle->phyAddr, PHY_BASICCONTROL_REG, PHY_BCTL_RESET_MASK);
//    do
//    {
//        MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_BASICCONTROL_REG, &reg);
//    } while (delay-- && reg & PHY_BCTL_RESET_MASK);

//    if (!delay)
//    {
//        return kStatus_Fail;
//    }

//    /* Set the ability. */
//    MDIO_Write(handle->mdioHandle, handle->phyAddr, PHY_AUTONEG_ADVERTISE_REG, (PHY_ALL_CAPABLE_MASK | 0x1U));

//    /* Start Auto negotiation and wait until auto negotiation completion */
//    MDIO_Write(handle->mdioHandle, handle->phyAddr, PHY_BASICCONTROL_REG,
//               (PHY_BCTL_AUTONEG_MASK | PHY_BCTL_RESTART_AUTONEG_MASK));
//    delay = PHY_TIMEOUT_COUNT;
//    do
//    {
//        MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_SEPCIAL_CONTROL_REG, &reg);
//        delay--;
//    } while (delay && ((reg & PHY_SPECIALCTL_AUTONEGDONE_MASK) == 0));

//    if (!delay)
//    {
//        return kStatus_Fail;
//    }

//    /* Waiting a moment for phy stable. */
//    for (delay = 0; delay < PHY_TIMEOUT_COUNT; delay++)
//    {
//        __ASM("nop");
//        PHY_GetLinkStatus(handle, &status);
//        if (status)
//        {
//            break;
//        }
//    }

    return kStatus_Success;
}

status_t PHY_LAN8720A_Write(phy_handle_t *handle, uint32_t phyReg, uint32_t data)
{
    return MDIO_Write(handle->mdioHandle, handle->phyAddr, phyReg, data);
}

status_t PHY_LAN8720A_Read(phy_handle_t *handle, uint32_t phyReg, uint32_t *dataPtr)
{
    return MDIO_Read(handle->mdioHandle, handle->phyAddr, phyReg, dataPtr);
}

status_t PHY_LAN8720A_GetLinkStatus(phy_handle_t *handle, bool *status)
{
    uint32_t reg;
    status_t result = kStatus_Success;

    /* Read the basic status register. */
    result = MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_BASICSTATUS_REG, &reg);
    if (result == kStatus_Success)
    {
        if (reg & PHY_BSTATUS_LINKSTATUS_MASK)
        {
            /* link up. */
            *status = true;
        }
        else
        {
            *status = false;
        }
    }
    return result;
}

status_t PHY_LAN8720A_GetLinkSpeedDuplex(phy_handle_t *handle, phy_speed_t *speed, phy_duplex_t *duplex)
{
    assert(duplex);
    assert(speed);

    uint32_t reg;
    status_t result = kStatus_Success;

    /* Read the control two register. */
    result = MDIO_Read(handle->mdioHandle, handle->phyAddr, PHY_SEPCIAL_CONTROL_REG, &reg);
    if (result == kStatus_Success)
    {
        if (reg & PHY_SPECIALCTL_DUPLEX_MASK)
        {
            /* Full duplex. */
            *duplex = kPHY_FullDuplex;
        }
        else
        {
            /* Half duplex. */
            *duplex = kPHY_HalfDuplex;
        }

        if (reg & PHY_SPECIALCTL_100SPEED_MASK)
        {
            /* 100M speed. */
            *speed = kPHY_Speed100M;
        }
        else
        { /* 10M speed. */
            *speed = kPHY_Speed10M;
        }
    }
    return result;
}
