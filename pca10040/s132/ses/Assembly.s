	.globl   compFilt
     .p2align 2
	.type    compFilt,%function
                
         //r0 -> gyro angle.
         //r1 -> acc angle
         //r2 -> last angle_estimate
compFilt:                     // Function "smuad" entry point.
	.fnstart
         LDR r3, =0x799A0666  //pack the filter constants in a register q0
         ADD r2, r0,r2 // add gyro and last value
         PKHBT r2 ,r1,r2 , LSL #16
         SMUAD r0,r3,r2
         ASR r0, #15
         BX       lr           // Return by branching to the address in the link register.
        .fnend