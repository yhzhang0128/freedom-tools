/*
 * QEMU RISC-V Board Compatible with SiFive Freedom E SDK
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * Provides a board compatible with the SiFive Freedom E SDK:
 *
 * 0) UART
 * 1) CLINT (Core Level Interruptor)
 * 2) PLIC (Platform Level Interrupt Controller)
 * 3) PRCI (Power, Reset, Clock, Interrupt)
 * 4) Registers emulated as RAM: AON, GPIO, QSPI, PWM
 * 5) Flash memory emulated as RAM
 *
 * The Mask ROM reset vector jumps to the flash payload at 0x2040_0000.
 * The OTP ROM and Flash boot code will be emulated in a future version.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/misc/unimp.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/sifive_e.h"
#include "hw/riscv/boot.h"
#include "hw/char/sifive_uart.h"
#include "hw/intc/sifive_clint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/misc/sifive_e_prci.h"
#include "hw/misc/sifive_test.h"
#include "chardev/char.h"
#include "sysemu/arch_init.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} sifive_e_memmap[] = {
    [SIFIVE_E_DEV_DEBUG] =    {        0x0,     0x1000 },
    [SIFIVE_E_DEV_MROM] =     {     0x1000,     0x2000 },
    [SIFIVE_E_DEV_OTP] =      {    0x20000,     0x2000 },
    [SIFIVE_E_DEV_TEST] =     {   0x100000,     0x1000 },
    [SIFIVE_E_DEV_CLINT] =    {  0x2000000,    0x10000 },

    [SIFIVE_E_DEV_ITIM] =     {  0x8000000,  0x2000000 },

    [SIFIVE_E_DEV_PLIC] =     {  0xc000000,  0x4000000 },
    [SIFIVE_E_DEV_AON] =      { 0x10000000,     0x8000 },
    [SIFIVE_E_DEV_PRCI] =     { 0x10008000,     0x8000 },
    [SIFIVE_E_DEV_OTP_CTRL] = { 0x10010000,     0x1000 },
    [SIFIVE_E_DEV_GPIO0] =    { 0x10012000,     0x1000 },
    [SIFIVE_E_DEV_UART0] =    { 0x10013000,     0x1000 },
    [SIFIVE_E_DEV_QSPI0] =    { 0x10014000,     0x1000 },
    [SIFIVE_E_DEV_PWM0] =     { 0x10015000,     0x1000 },
    [SIFIVE_E_DEV_UART1] =    { 0x10023000,     0x1000 },
    [SIFIVE_E_DEV_QSPI1] =    { 0x10024000,     0x1000 },
    [SIFIVE_E_DEV_PWM1] =     { 0x10025000,     0x1000 },
    [SIFIVE_E_DEV_QSPI2] =    { 0x10034000,     0x1000 },
    [SIFIVE_E_DEV_PWM2] =     { 0x10035000,     0x1000 },
    [SIFIVE_E_DEV_XIP] =      { 0x20000000, 0x20000000 },
    [SIFIVE_E_DEV_DTIM] =     { 0x80000000,   0x400000 }
};

static void sifive_e_machine_init(MachineState *machine)
{
    const struct MemmapEntry *memmap = sifive_e_memmap;

    SiFiveEState *s = RISCV_E_MACHINE(machine);
    MemoryRegion *sys_mem = get_system_memory();
    MemoryRegion *main_mem = g_new(MemoryRegion, 1);
    int i;

    /* Initialize SoC */
    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_RISCV_E_SOC);
    qdev_realize(DEVICE(&s->soc), NULL, &error_abort);

    /* Data Tightly Integrated Memory */
    memory_region_init_ram(main_mem, NULL, "riscv.sifive.e.ram",
        memmap[SIFIVE_E_DEV_DTIM].size, &error_fatal);
    memory_region_add_subregion(sys_mem,
        memmap[SIFIVE_E_DEV_DTIM].base, main_mem);

    /* Instruction Tightly Integrated Memory */
    MemoryRegion *itim_mem = g_new(MemoryRegion, 1);
    memory_region_init_ram(itim_mem, NULL, "riscv.sifive.e.itim",
        memmap[SIFIVE_E_DEV_ITIM].size, &error_fatal);
    memory_region_add_subregion(sys_mem,
        memmap[SIFIVE_E_DEV_ITIM].base, itim_mem);

    /* Mask ROM reset vector */
    uint32_t reset_vec[4];

    if (s->revb) {
        reset_vec[1] = 0x200102b7;  /* 0x1004: lui     t0,0x20010 */
    } else {
        reset_vec[1] = 0x204002b7;  /* 0x1004: lui     t0,0x20400 */
    }
    reset_vec[2] = 0x00028067;      /* 0x1008: jr      t0 */

    reset_vec[0] = reset_vec[3] = 0;

    /* copy in the reset vector in little_endian byte order */
    for (i = 0; i < sizeof(reset_vec) >> 2; i++) {
        reset_vec[i] = cpu_to_le32(reset_vec[i]);
    }
    rom_add_blob_fixed_as("mrom.reset", reset_vec, sizeof(reset_vec),
                          memmap[SIFIVE_E_DEV_MROM].base, &address_space_memory);

    if (machine->kernel_filename) {
        riscv_load_kernel(machine->kernel_filename,
                          memmap[SIFIVE_E_DEV_DTIM].base, NULL);
    }
}

static bool sifive_e_machine_get_revb(Object *obj, Error **errp)
{
    SiFiveEState *s = RISCV_E_MACHINE(obj);

    return s->revb;
}

static void sifive_e_machine_set_revb(Object *obj, bool value, Error **errp)
{
    SiFiveEState *s = RISCV_E_MACHINE(obj);

    s->revb = value;
}

static void sifive_e_machine_instance_init(Object *obj)
{
    SiFiveEState *s = RISCV_E_MACHINE(obj);

    s->revb = false;
}

static void sifive_e_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V Board compatible with SiFive E SDK";
    mc->init = sifive_e_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = SIFIVE_E_CPU;

    object_class_property_add_bool(oc, "revb", sifive_e_machine_get_revb,
                                   sifive_e_machine_set_revb);
    object_class_property_set_description(oc, "revb",
                                          "Set on to tell QEMU that it should model "
                                          "the revB HiFive1 board");
}

static const TypeInfo sifive_e_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("sifive_e"),
    .parent     = TYPE_MACHINE,
    .class_init = sifive_e_machine_class_init,
    .instance_init = sifive_e_machine_instance_init,
    .instance_size = sizeof(SiFiveEState),
};

static void sifive_e_machine_init_register_types(void)
{
    type_register_static(&sifive_e_machine_typeinfo);
}

type_init(sifive_e_machine_init_register_types)

static void sifive_e_soc_init(Object *obj)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    SiFiveESoCState *s = RISCV_E_SOC(obj);

    object_initialize_child(obj, "cpus", &s->cpus, TYPE_RISCV_HART_ARRAY);
    object_property_set_int(OBJECT(&s->cpus), "num-harts", ms->smp.cpus,
                            &error_abort);
    object_property_set_int(OBJECT(&s->cpus), "resetvec", 0x1004, &error_abort);
    object_initialize_child(obj, "riscv.sifive.e.gpio0", &s->gpio,
                            TYPE_SIFIVE_GPIO);
}

#define BLOCK_SIZE        512
#define SPI1_TXDATA       72UL
#define SPI1_RXDATA       76UL
#define TYPE_EGOS_SDCARD "egos.2000.sd"
enum {
    SD_IDLE,
    SD_READY,
    SD_GETTING_CMD0,
    SD_GETTING_CMD8,
    SD_GETTING_CMD16,
    SD_GETTING_CMD55,
    SD_GETTING_CMD58,
    SD_GETTING_ACMD41,
    SD_GETTING_CMD17,  // read single block
    SD_GETTING_CMD24,  // write single block
    SD_FINISH_CMD24
};
static int sd_state;
static int sd_cmd_idx;
static int sd_cmd24_idx;
static char sd_cmd[32];
static char sd_storage[4 * 1024 * 1024];

// SD commands for initialization
// cmd0 {0x40, 0x00, 0x00, 0x00, 0x00, 0x95} => 0x01
// cmd8 {0x48, 0x00, 0x00, 0x01, 0xAA, 0x87} => 0x01
// cmd16 = {0x50, 0x00, 0x00, 0x02, 0x00, 0xFF} => 0x00
// cmd58 = {0x7A, 0x00, 0x00, 0x00, 0x00, 0xFF} => 0xC0FF8000
// cmd55 = {0x77, 0x00, 0x00, 0x00, 0x00, 0xFF} => 0x00
// acmd41 = {0x69, 0x40, 0x00, 0x00, 0x00, 0xFF} => 0x00

// SD commands for read and write
// cmd17 = {0x51, arg[3], arg[2], arg[1], arg[0], 0xFF} => read single block
// cmd24 = {0x58, arg[3], arg[2], arg[1], arg[0], 0xFF} => write single block

static uint64_t egos_sd_mmio_read(void *storage, hwaddr addr, unsigned int size)
{
    //printf("[QEMU] egos_sd_mmio_read: addr=%ld, size=%d, state=%d\n", addr, size, sd_state);
    if (addr != SPI1_RXDATA) return 0;

    if (sd_state == SD_FINISH_CMD24) {
        // ack for disk block write
        sd_cmd_idx = 0;
        sd_cmd24_idx = 0;
        sd_state = SD_IDLE;
        // emulate 30ms disk latency
        usleep(30 * 1000);
        return 0x05;
    }
    if (sd_state != SD_READY) {
        if (sd_cmd_idx >= 6) sd_state = SD_READY;
        return 0xFF;
    }

    // sd_cmd should hold a complete SD command
    uint64_t ret = 0xFF;
    static int cmd8_idx = 0, cmd58_idx = 0;
    static char cmd58_reply[] = {0x00, 0xC0, 0xFF, 0x80, 0x00};
    static char cmd8_reply[] = {0x01, 0x00, 0x00, 0x01, 0xAA};

    static int cmd17_idx = -1;
    static char block_to_read[BLOCK_SIZE + 1];
    block_to_read[0] = 0xFE;

    switch (sd_cmd[0]) {
    case 0x40: // cmd0
        ret = 0x01;
        break;
    case 0x48: // cmd8
        ret = cmd8_reply[cmd8_idx++];
        break;
    case 0x7A: // cmd58
        ret = cmd58_reply[cmd58_idx++];
        break;
    case 0x50: // cmd16
    case 0x51: // cmd17
    case 0x58: // cmd24
    case 0x69: // acmd41
    case 0x77: // cmd55
        ret = 0x00;
        break;
    default:
        printf("[QEMU] unknown SD command type=0x%x\n", sd_cmd[0]);
        assert(0);
    }

    if (sd_cmd[0] == 0x51 && cmd17_idx == -1) {
        // before sending a disk block
        cmd17_idx = 0;
        int part1 = (sd_cmd[1] << 24) & 0xFF000000;
        int part2 = (sd_cmd[2] << 16) & 0x00FF0000;
        int part3 = (sd_cmd[3] << 8)  & 0x0000FF00;
        int part4 = sd_cmd[4]         & 0x000000FF;
        int block_no = part1 | part2 | part3 | part4;

        // emulate 30ms disk latency
        usleep(30 * 1000);

        char *dst = block_to_read + 1;
        memcpy(dst, sd_storage + block_no * BLOCK_SIZE, BLOCK_SIZE);
    } else if (sd_cmd[0] == 0x51 && cmd17_idx < BLOCK_SIZE) {
        // during sending a disk block
        ret = block_to_read[cmd17_idx++];
    } else if (sd_cmd[0] == 0x51 && cmd17_idx == BLOCK_SIZE) {
        // sending the last byte of a disk block
        ret = block_to_read[cmd17_idx];
        cmd17_idx = -1;
        sd_cmd_idx = 0;
        sd_state = SD_IDLE;
    } else if (sd_cmd[0] == 0x7A && cmd58_idx == 5) {
        cmd58_idx = 0;
        sd_cmd_idx = 0;
        sd_state = SD_IDLE;
    } else if (sd_cmd[0] == 0x48 && cmd8_idx == 5) {
        cmd8_idx = 0;
        sd_cmd_idx = 0;
        sd_state = SD_IDLE;
    } else if (sd_cmd[0] != 0x7A && sd_cmd[0] != 0x48
            && sd_cmd[0] != 0x51 && sd_cmd[0] != 0x58) {
        sd_cmd_idx = 0;
        sd_state = SD_IDLE;
    }

    ret &= 0xFF;
    return ret;
}

static void egos_sd_mmio_write(void *storage, hwaddr addr,
                               uint64_t val64, unsigned int size)
{
    //printf("[QEMU] egos_sd_mmio_write: addr=%ld, val=0x%lx, size=%d, state=%d\n", addr, val64, size, sd_state);
    if (addr != SPI1_TXDATA) return;

    if (sd_state != SD_IDLE && sd_state != SD_READY) {
        sd_cmd[sd_cmd_idx++] = (char)val64;
        if (sd_cmd_idx == 32) assert(0);
        return;
    }

    if (sd_state == SD_READY && sd_cmd[0] == 0x58) {
        // write a disk block
        int part1 = (sd_cmd[1] << 24) & 0xFF000000;
        int part2 = (sd_cmd[2] << 16) & 0x00FF0000;
        int part3 = (sd_cmd[3] << 8)  & 0x0000FF00;
        int part4 = sd_cmd[4]         & 0x000000FF;
        int block_no = part1 | part2 | part3 | part4;

        if (sd_cmd24_idx == 0) {
            // token
            if (val64 == 0xFE) {
                //printf("[QEMU] get token 0x%lx for cmd24\n", val64);
                sd_cmd24_idx++;
            }
        } else if (sd_cmd24_idx <= BLOCK_SIZE) {
            // disk block
            sd_storage[block_no * BLOCK_SIZE + sd_cmd24_idx - 1] = (char)val64;
            sd_cmd24_idx++;
        } else if (sd_cmd24_idx <= BLOCK_SIZE + 2) {
            // checksum
            sd_cmd24_idx++;
        } else {
            sd_state = SD_FINISH_CMD24;
        }
        return;
    }

    switch (val64) {
    case 0x40: // cmd0
        sd_cmd[sd_cmd_idx++] = 0x40;
        sd_state = SD_GETTING_CMD0;
        break;
    case 0x48: // cmd8
        sd_cmd[sd_cmd_idx++] = 0x48;
        sd_state = SD_GETTING_CMD8;
        break;
    case 0x50: // cmd16
        sd_cmd[sd_cmd_idx++] = 0x50;
        sd_state = SD_GETTING_CMD16;
        break;
    case 0x51: // cmd17
        sd_cmd[sd_cmd_idx++] = 0x51;
        sd_state = SD_GETTING_CMD17;
        break;
    case 0x58: // cmd24
        sd_cmd[sd_cmd_idx++] = 0x58;
        sd_state = SD_GETTING_CMD24;
        break;
    case 0x69:
        sd_cmd[sd_cmd_idx++] = 0x69;
        sd_state = SD_GETTING_ACMD41;
        break;
    case 0x77:
        sd_cmd[sd_cmd_idx++] = 0x77;
        sd_state = SD_GETTING_CMD55;
        break;
    case 0x7A: // cmd58
        sd_cmd[sd_cmd_idx++] = 0x7A;
        sd_state = SD_GETTING_CMD58;
        break;
    case 0xFF:
        break;
    default:
        printf("[QEMU] unknown SD command type=0x%lx\n", val64);
        assert(0);
    }
}

static const MemoryRegionOps egos_sd_ops = {
    .read = egos_sd_mmio_read,
    .write = egos_sd_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#include <fcntl.h>
#include <sys/stat.h>
static void egos_sd_reset(DeviceState *d)
{
    struct stat st;
    const char* file_name = "tools/disk.img";
    stat(file_name, &st);
    if (st.st_size != sizeof(sd_storage)) {
        printf("[QEMU] tools/disk.img is %ld instead of %ld bytes\n", st.st_size, sizeof(sd_storage));
        printf("[QEMU] Make sure to `make install` first\n");
        assert(0);
    }

    int fd = open(file_name, O_RDONLY);
    for (int nread = 0; nread < st.st_size; )
        nread += read(fd, sd_storage + nread, st.st_size - nread);
}

static void egos_sd_realize(DeviceState *dev, Error **errp)
{
    egos_sd_reset(dev);

    const MemMapEntry *memmap = sifive_e_memmap;
    MemoryRegion *spi1_mmio = g_new(MemoryRegion, 1);
    memory_region_init_io(spi1_mmio, OBJECT(dev),
                          &egos_sd_ops, sd_storage,
                          TYPE_EGOS_SDCARD,
                          memmap[SIFIVE_E_DEV_QSPI1].size);

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), spi1_mmio);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, memmap[SIFIVE_E_DEV_QSPI1].base);
}

static void egos_sd_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = egos_sd_reset;
    dc->realize = egos_sd_realize;
}

static const TypeInfo egos_sd_info = {
    .name           = TYPE_EGOS_SDCARD,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(sd_storage),
    .class_init     = egos_sd_init,
};

static void egos_sd_register_types(void)
{
    type_register_static(&egos_sd_info);
}
type_init(egos_sd_register_types)

static void sifive_e_soc_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    const struct MemmapEntry *memmap = sifive_e_memmap;
    SiFiveESoCState *s = RISCV_E_SOC(dev);
    MemoryRegion *sys_mem = get_system_memory();

    object_property_set_str(OBJECT(&s->cpus), "cpu-type", ms->cpu_type,
                            &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->cpus), &error_abort);

    /* Mask ROM */
    memory_region_init_rom(&s->mask_rom, OBJECT(dev), "riscv.sifive.e.mrom",
                           memmap[SIFIVE_E_DEV_MROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem,
        memmap[SIFIVE_E_DEV_MROM].base, &s->mask_rom);

    /* MMIO */
    s->plic = sifive_plic_create(memmap[SIFIVE_E_DEV_PLIC].base,
        (char *)SIFIVE_E_PLIC_HART_CONFIG, 0,
        SIFIVE_E_PLIC_NUM_SOURCES,
        SIFIVE_E_PLIC_NUM_PRIORITIES,
        SIFIVE_E_PLIC_PRIORITY_BASE,
        SIFIVE_E_PLIC_PENDING_BASE,
        SIFIVE_E_PLIC_ENABLE_BASE,
        SIFIVE_E_PLIC_ENABLE_STRIDE,
        SIFIVE_E_PLIC_CONTEXT_BASE,
        SIFIVE_E_PLIC_CONTEXT_STRIDE,
        memmap[SIFIVE_E_DEV_PLIC].size);
    sifive_clint_create(memmap[SIFIVE_E_DEV_CLINT].base,
        memmap[SIFIVE_E_DEV_CLINT].size, 0, ms->smp.cpus,
        SIFIVE_SIP_BASE, SIFIVE_TIMECMP_BASE, SIFIVE_TIME_BASE,
        SIFIVE_CLINT_TIMEBASE_FREQ, false);
    create_unimplemented_device("riscv.sifive.e.aon",
        memmap[SIFIVE_E_DEV_AON].base, memmap[SIFIVE_E_DEV_AON].size);
    sifive_e_prci_create(memmap[SIFIVE_E_DEV_PRCI].base);

    /* GPIO */

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }

    /* Map GPIO registers */
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, memmap[SIFIVE_E_DEV_GPIO0].base);

    /* Pass all GPIOs to the SOC layer so they are available to the board */
    qdev_pass_gpios(DEVICE(&s->gpio), dev, NULL);

    /* Connect GPIO interrupts to the PLIC */
    for (int i = 0; i < 32; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), i,
                           qdev_get_gpio_in(DEVICE(s->plic),
                                            SIFIVE_E_GPIO0_IRQ0 + i));
    }

    sifive_uart_create(sys_mem, memmap[SIFIVE_E_DEV_UART0].base,
        serial_hd(0), qdev_get_gpio_in(DEVICE(s->plic), SIFIVE_E_UART0_IRQ));
    create_unimplemented_device("riscv.sifive.e.qspi0",
        memmap[SIFIVE_E_DEV_QSPI0].base, memmap[SIFIVE_E_DEV_QSPI0].size);
    create_unimplemented_device("riscv.sifive.e.pwm0",
        memmap[SIFIVE_E_DEV_PWM0].base, memmap[SIFIVE_E_DEV_PWM0].size);
    sifive_uart_create(sys_mem, memmap[SIFIVE_E_DEV_UART1].base,
        serial_hd(1), qdev_get_gpio_in(DEVICE(s->plic), SIFIVE_E_UART1_IRQ));
    /* create_unimplemented_device("riscv.sifive.e.qspi1", */
    /*     memmap[SIFIVE_E_DEV_QSPI1].base, memmap[SIFIVE_E_DEV_QSPI1].size); */
    /* Map SPI1 as an SD card device */
    DeviceState *sd = qdev_new(TYPE_EGOS_SDCARD);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sd), &error_fatal);

    create_unimplemented_device("riscv.sifive.e.pwm1",
        memmap[SIFIVE_E_DEV_PWM1].base, memmap[SIFIVE_E_DEV_PWM1].size);
    create_unimplemented_device("riscv.sifive.e.qspi2",
        memmap[SIFIVE_E_DEV_QSPI2].base, memmap[SIFIVE_E_DEV_QSPI2].size);
    create_unimplemented_device("riscv.sifive.e.pwm2",
        memmap[SIFIVE_E_DEV_PWM2].base, memmap[SIFIVE_E_DEV_PWM2].size);
    sifive_test_create(memmap[SIFIVE_E_DEV_TEST].base);

    /* Flash memory */
    memory_region_init_rom(&s->xip_mem, OBJECT(dev), "riscv.sifive.e.xip",
                           memmap[SIFIVE_E_DEV_XIP].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[SIFIVE_E_DEV_XIP].base,
        &s->xip_mem);
}

static void sifive_e_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sifive_e_soc_realize;
    /* Reason: Uses serial_hds in realize function, thus can't be used twice */
    dc->user_creatable = false;
}

static const TypeInfo sifive_e_soc_type_info = {
    .name = TYPE_RISCV_E_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SiFiveESoCState),
    .instance_init = sifive_e_soc_init,
    .class_init = sifive_e_soc_class_init,
};

static void sifive_e_soc_register_types(void)
{
    type_register_static(&sifive_e_soc_type_info);
}

type_init(sifive_e_soc_register_types)
