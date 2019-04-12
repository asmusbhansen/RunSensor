        .syntax unified
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

.globl   pATAN2
.p2align 2
.type    pATAN2,%function 
.thumb


pATAN2:
        .fnstart
         //put abs af y i r2

         MOV r2,r0
         CMP     r2, #0
         IT LT 
         RSBLT  r2, r2, #0
         
         // put abs af x i r3
         MOV r3,r1
         CMP  r3, #0
         IT LT
         RSBLT   r3, r3, #0

         //---------------perform -90 deg rotation 
         //compare |y|,|x|
         CMP r2, r3 // compare
         // r2, r3 is now free

         MOV r3, #0    // r3 is now used for storing the rotation to add in the end
         BLE rotation //jump if y is less than 

         MOV r2,r1 // save x temporarely
         MOV r1,r0 // save y in x
         RSB r0,r2,#0 // save -x in y
         
         //load the 90 degrees into add register 
         MOV r3, #16383

         // starting rotation pjat
        
rotation:
         MOV r2, #0x7fff// store 180 degree turn in r2
         CMP r1,#0 //check if x is positive
         BGT angcalc// if so skip to angle calculation
        

         //if x<0 and y>=0 iverted implementation 
         CMP r1, #0
         BGE test3 //x must be zero so skip to test3
         CMP r0, #0
         BLT test2 //skip if less than

         ADD r3, r3, r2 // turn 180 degrees
         B angcalc // jump to angle calculation
test2:
         //if x<0 and y<0 

         SUB r3, r3, r2 // turn -180 degrees
         B angcalc // jump to angle calculation

test3:
         //if x= 0 and y > 0 / y<0
         //at this point x is 0 which means only y has to be checked
         CMP r0 , #0
         MOV r0,#16383
         
         IT LE
         RSBLE r0,r0,#0
         
         ADD r0,r0,r3 // calculate angle
         BX lr // return angle

angcalc:
          //r0 = x, r1 = y , r2 = 0x7fff, r3 = angle turn
         
          //divide y/x in a fixed point manner 
         LSL r0, #15
         SDIV r0, r0,r1 
         
         CMP r0,#0x8000 // this happens 
         IT EQ 
         SUBEQ r0,#1

         //get absolute value of input to atan save to r1
         MOV r1 , r0 
         CMP     r1, #0
         IT LT
         RSBLT   r1, r1, #0
         
         //calc (1-|x|)
         RSB r1,r1,r2
         //calc x(1-|x|) but in q30
         MUL r1,r0,r1
         
        // constants to r2  [(0.273/pi) 0.25] in two instructions 1 was not possible 
         MOVW r2,  #0x2000
         MOVT r2, #0x0B1F 
         
         PKHBT r0, r0, r1, LSL #1 //pack r0 in the bottom and r1 shifted left 1 in the top

         SMUAD r0,r0,r2 //calculate the angle
         ASR r0,#15 // shift to fit 16 bit
   
         SXTAH r0, r0, r3 //add the angle necesary for rotation 
         BX lr           // Return by branching to the address in the link register.
 .fnend