.syntax unified
.cpu cortex-m4
.thumb

.text
    .global fpu_enable

fpu_enable:
    push  {r0, r1, lr}
    ldr.w r0, =0xE000ED88
    ldr   r1, [r0]
    orr   r1, r1, #(0xF << 20)
    str   r1, [r0]
    dsb
    isb
    pop   {r0, r1, pc}
