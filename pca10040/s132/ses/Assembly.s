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

.globl   fATAN2
.p2align 2
.type    fATAN2,%function 
fATAN2:
        .fnstart
         //put abs af y i r2
         cmp     r0, #0
         rsblt   r2, r0, #0
         // put abs af x i r3
         cmp     r1, #0
         rsblt   r3, r1, #0

         //---------------perform 90 deg rotation 
         //compare |y|,|x|
         cmp r2, r3 
         // r2, r3 is now free
         //if y greater than x
         movGT r2,r1 // save x temporarely
         movGT r1,r0 // save y in x
         rsbGT r0,r2,#0 // save -x in y
         
         //load r2 with add value depending on whether or not a rotation was performed
         LDRGT r2, #16384
         LDRLEQ r2, #0

         //divide y/x
         ASL R0, #16
         sdiv r0, r0,r1
 
         
         
         BX       lr           // Return by branching to the address in the link register.
 .fnend
