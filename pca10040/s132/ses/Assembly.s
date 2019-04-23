        .syntax unified
	.globl   compFilt
        .p2align 2
	.type    compFilt,%function
                
         //r0 -> gyro angle.
         //r1 -> acc angle
         //r2 -> last angle_estimate
compFilt:                     // Function "smuad" entry point.
	.fnstart
         LDR r3, =0x7FC4003B  //pack the filter constants in a register q0
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




//.AREA    MyData, DATA, READWRITE
.text
.syntax unified
.globl   sDFT
.p2align 2
.type    sDFT,%function 
.thumb

.extern trigValues
sDFT:
        .fnstart
        push {r4-r12} // store unsafe registers

        // load oldest value and save newest.  
        // load circular pointer
        LDR r4, =circPoint;   //get address pointer 
        LDR r2, [r4]; //load value on address

        LDR r3, =circBuff; //get buffer pointer
        LDR r1,[r3, r2,lsl #2];  //get old value in the new spot (sh signed halfword)
        ASR r0,#10

        STR r0,[r3,r2,lsl #2]; // save new  value in the new spot
        
        //move  circpoint forward
        ADD r2,#1     
        //wrap if above #1023
        MOV r3, 0x03FF
        AND r2, r3

        STR r2, [r4] // save new buffer pointer to ram
        SUB r0,r1 //create xnew-xold
        
        //buffer handling completed r0, r1 hold new and old value
        //_____________________________________________________________
        
        LDR r2, =trigValues //store trigvalues entry bottom cos top sin
        LDR r3, =dftBins //store bin entry bottom real,top img
        MOV r4,#0     //is offset
        MOV r5,#0     // is max bin. //also what needs to be pushed out later
        MOV r6,#0     // is max value
        //r0 is the combined newest and oldest value
        //r2 and r3 are pointers to the trigonometric values
        //max and bin
top:    
        LDR r7, [r3,r4,lsl #2] //load bins
        SADD16 R8,r0, r7 
        PKHBT r7,r8,r7

        LDR r8, [r2,r4,lsl #2] //load trig values
        
        //calculate img bin
        SMUSDX r9, r8,r7 //i*cos - r*sin 



        //calculate real bin
        SMUAD r10, r7, r8 // cos*r + i *sin
        ASR   r10, #15
                
        PKHBT r11, r10, r9, lsl #1 // pack the the new bins
        
        STR r11,[r3,r4,lsl #2]     // save to bin

        //find largest bin.
        SMUAD r11,r11,r11 // real^2 + img^2

        //no need to shift as we are finding max largest value

        CMP r11, r6 // if new max is greater than old max
        ITT GT
        MOVGT r6, r11 //save new max
        MOVGT r5, r4 // and its bin

        //update pointer
        ADD r4, #1 
        CMP r4,#50;
        BNE top // end of loop
       
        MOV r0, r5
        
        //LDR r4 , =circPoint
        //LDR r0 , [r4]
        // end of program we done
        pop {r4-r12} // restore unsafe registers
        BX lr           // Return by branching to the address in the link register.
 .fnend
 .LTORG

.bss //ensure that it is in ram
 .p2align 2
circPoint: .space 4
circBuff: .space 1024 *4 //
dftBins: .space 60 *4 // bins