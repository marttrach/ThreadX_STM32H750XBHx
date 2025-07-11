.syntax unified
.cpu cortex-m7
.fpu fpv5-d16
.thumb

.global g_pfnVectors
.global Reset_Handler

.extern SystemInit
.extern main

/* Vector table */
.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
g_pfnVectors:
    .word _estack
    .word Reset_Handler
    /* Core handlers */
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler
    /* Device specific interrupts - all point to Default_Handler */
    .rept 240-16
    .word Default_Handler
    .endr

/* Reset handler */
.section .text.Reset_Handler,"ax",%progbits
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =_estack
    mov sp, r0
    /* Copy data from flash to RAM */
    ldr r0, =_sdata
    ldr r1, =_etext
    ldr r2, =_edata
1:
    cmp r0, r2
    bge 2f
    ldr r3, [r1], #4
    str r3, [r0], #4
    b 1b
2:
    /* Zero BSS */
    ldr r0, =_sbss
    ldr r1, =_ebss
    mov r2, #0
3:
    cmp r0, r1
    bge 4f
    str r2, [r0], #4
    b 3b
4:
    bl SystemInit
    bl main
    b .

.size Reset_Handler, .-Reset_Handler

/* Default handler */
.section .text.Default_Handler,"ax",%progbits
.type Default_Handler, %function
Default_Handler:
    b .
.size Default_Handler, .-Default_Handler

/* Dummy handlers for core exceptions */
.section .text.Handlers,"ax",%progbits
NMI_Handler:           b .
HardFault_Handler:     b .
MemManage_Handler:     b .
BusFault_Handler:      b .
UsageFault_Handler:    b .
SVC_Handler:           b .
DebugMon_Handler:      b .
PendSV_Handler:        b .
SysTick_Handler:       b .

