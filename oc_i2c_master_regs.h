#ifndef _OC_I2C_MASTER_REGS_H_
#define _OC_I2C_MASTER_REGS_H_

#define OC_I2C_MASTER_CTR_CORE_EN   (0x80)
#define OC_I2C_MASTER_CTR_IRQ_EN    (0x40)

#define OC_I2C_MASTER_CR_STA   (0x80)
#define OC_I2C_MASTER_CR_STO   (0x40)
#define OC_I2C_MASTER_CR_RD    (0x20)
#define OC_I2C_MASTER_CR_WR    (0x10)
#define OC_I2C_MASTER_CR_ACK   (0x08)
#define OC_I2C_MASTER_CR_IACK  (0x01)

#define OC_I2C_MASTER_SR_RxACK (0x80)
#define OC_I2C_MASTER_SR_BUSY  (0x40)
#define OC_I2C_MASTER_SR_AL    (0x20)
#define OC_I2C_MASTER_SR_TIP   (0x02)
#define OC_I2C_MASTER_SR_IF    (0x01)

#define I2C_SLAVE_ADDR 0x55

#endif /* _OC_I2C_MASTER_REGS_H_ */
