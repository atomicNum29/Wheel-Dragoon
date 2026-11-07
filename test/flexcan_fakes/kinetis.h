#pragma once
#include <stdint.h>

// IFLAG is write-one-to-clear, not a normal RAM assignment.
void fake_can_flags_cleared(uint32_t mask);
struct FakeCanFlags {
    volatile uint32_t value = 0;
    uint32_t writes = 0;
    operator volatile uint32_t &() { return value; }
    FakeCanFlags &operator=(uint32_t mask) {
        value &= ~mask;
        ++writes;
        fake_can_flags_cleared(mask);
        return *this;
    }
};
extern FakeCanFlags CAN0_IFLAG1;
extern volatile uint32_t fake_can_registers[14];
uint32_t fake_can_read_timer();
#define CAN0_TIMER fake_can_read_timer()
#define SIM_SCGC5 fake_can_registers[0]
#define SIM_SCGC6 fake_can_registers[1]
#define OSC0_CR fake_can_registers[2]
#define PORTA_PCR12 fake_can_registers[3]
#define PORTA_PCR13 fake_can_registers[4]
#define CAN0_CTRL1 fake_can_registers[5]
#define CAN0_MCR fake_can_registers[6]
#define CAN0_RXMGMASK fake_can_registers[7]
#define CAN0_RX14MASK fake_can_registers[8]
#define CAN0_RX15MASK fake_can_registers[9]
#define CAN0_IMASK1 fake_can_registers[10]
#define CAN0_CTRL2 fake_can_registers[11]
#define CAN0_RXFGMASK fake_can_registers[12]
#define IRQ_CAN_MESSAGE 29
extern bool fake_can_irq_enabled;
extern uint8_t fake_can_irq_priority;
extern void (*fake_can_irq_handler)();
#define NVIC_DISABLE_IRQ(irq) (fake_can_irq_enabled = false)
#define NVIC_ENABLE_IRQ(irq) (fake_can_irq_enabled = true)
#define NVIC_CLEAR_PENDING(irq) ((void)0)
#define NVIC_SET_PRIORITY(irq, priority) (fake_can_irq_priority = (priority))
inline void attachInterruptVector(int, void (*handler)()) { fake_can_irq_handler = handler; }
#define SIM_SCGC5_PORTA (1u << 9)
#define SIM_SCGC6_FLEXCAN0 (1u << 4)
#define OSC_ERCLKEN (1u << 7)
#define PORT_PCR_MUX(value) ((value) << 8)
