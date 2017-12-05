//*****************************************
//***    **********************************
//**  ***  *******     ********  *   ******
//**  **  *****    ****   *****  **  ******
//**  *  ******   ******  *****   *********
//**  **  *****   ******  *****  **  ******
//**  ***  ****    ****  ******  *   ******
//**  ****  ******      *******     *******
//*****************************************


#ifndef ROBOTICARM_H
#define ROBOTICARM_H


#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#include "PIDController.h"



  class roboticarm : 
                         
                   public Test


  {

//-----puntatori (braccio1.braccio2.braccio3.mano1dito1,mano2.dito2,mass,corpo-centrale)
b2Body* mass;
b2Body* body_center;
b2Body* right_arm_upper;
b2Body* right_arm_lower;
b2Body* right_arm_middle;
b2Body* right_hand_right;
b2Body* right_hand_left;
b2Body* right_finger_left;
b2Body* right_finger_right;
b2RevoluteJoint* joint1;
b2RevoluteJoint* joint2;
b2RevoluteJoint* joint3;

// PID controller (3 pid Controller for each joint) 
PIDController pid;
PIDController pid2;
PIDController pid3;
PIDController pid4;
PIDController pid5;
PIDController pid6;
PIDController pid7;



// angoli misurati
float actualAnglejoint1;
float actualAnglejoint2;
float actualAnglejoint3;
float actualAnglejoint4; //referred to finger1 left-right 
float actualAnglejoint5; //referred to finger2 left-right
//torque fornita ai motori
float forceVal;
float forceVal2;
float forceVal3;
//angoli desiderati 
float desiredAngle;
float desiredAngle2;
float desiredAngle3;
float desiredAngle4;//referred to finger1 left-right
float desiredAngle5;//referred to finger2  left-righet

// variabili per contatore
float i;
float c;
//booleane
bool torqueOn;
bool addAngle;
bool lessAngle;

public:



   roboticarm() {


                  torqueOn =true;
                  addAngle =false;
                  lessAngle =false;

                  pid = PIDController();
                  pid.setGains(0.6*6,2*6/6,6*6/8);
                  pid2 = PIDController();
                  pid2.setGains(0.6*6,2*6/3,6*3/8);
                  pid3 = PIDController();
                  pid3.setGains(0.6*7,2*7/3,7*3/8);
                  pid4 = PIDController();
                  pid4.setGains(0.6*7,1.2*8/2,2*7/8);
                  pid5 = PIDController();
                  pid5.setGains(0,0,0);
/*
                  pid = PIDController();
                  pid.setGains(0.6*7,1.2*7/5,5*7/8);
                  pid2 = PIDController();
                  pid2.setGains(0.6*7,1.2*7/5,3*7/8);
                  pid3 = PIDController();
                  pid3.setGains(0.6*7,1.2*8/6,4*7/8);

*/
/*
                  pid = PIDController();
                  pid.setGains(0.6*7,1.2*6/10,10*7/8);
                  pid2 = PIDController();
                  pid2.setGains(0.6*6,1.2*6/100,100*6/8);
                  pid3 = PIDController();
                  pid3.setGains(0.6*5,1.2*5/100,100*5/8);

*/


                  desiredAngle = 0*DEGTORAD;
                  desiredAngle2 = 0*DEGTORAD;
                  desiredAngle3 = 0*DEGTORAD;

//-----------------------COMMON DEFINITIONS

                  //----------Variables
                  b2BodyDef bodyDef;
                  b2FixtureDef fixtureDef;
                  b2PolygonShape polygonShape;
                  b2RevoluteJointDef revoluteJointDef;




                 //-------------Blocco Statico 
                  bodyDef.type = b2_staticBody;
                  fixtureDef.density = 1;
                  polygonShape.SetAsBox(3,3);
                  fixtureDef.shape = &polygonShape;
                  bodyDef.position.Set(0,30);
                  

                  //----CENTRAL BODY-----



                  body_center = m_world->CreateBody( &bodyDef );
                  body_center->CreateFixture( &fixtureDef );
                  body_center->SetGravityScale(0);
                  bodyDef.type = b2_dynamicBody;



                  //----RIGHT ARM UPPER-----



                  bodyDef.position.Set(10,30);
                  polygonShape.SetAsBox(4.5,0.7);

                  right_arm_upper = m_world->CreateBody( &bodyDef );
                  right_arm_upper->CreateFixture( &fixtureDef );
                  fixtureDef.density=1;
                  right_arm_upper->SetGravityScale(0);

                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-4,0);
                  revoluteJointDef.bodyA = body_center;
                  revoluteJointDef.bodyB = right_arm_upper;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -180 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  joint1 = (b2RevoluteJoint*) m_world->CreateJoint( &revoluteJointDef );

             

                // Revolute Joint  


                  bodyDef.position.Set(16,30);
                  polygonShape.SetAsBox(3,0.7);

                  right_arm_middle = m_world->CreateBody( &bodyDef );
                  right_arm_middle->CreateFixture( &fixtureDef );
                  fixtureDef.density=1;
                 right_arm_middle->SetGravityScale(0);

                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-4,0);
                  revoluteJointDef.bodyA = right_arm_upper;
                  revoluteJointDef.bodyB = right_arm_middle;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  joint2 = (b2RevoluteJoint*) m_world->CreateJoint( &revoluteJointDef );


                  //----RIGHT ARM LOWER-----
                  bodyDef.position.Set(22,30);
                  polygonShape.SetAsBox(4,0.7);

                  right_arm_lower = m_world->CreateBody( &bodyDef );
                  right_arm_lower->CreateFixture( &fixtureDef );
                  right_arm_lower->SetGravityScale(0);
                                    fixtureDef.density=1;
                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-4,0);
                  revoluteJointDef.bodyA = right_arm_middle;
                  revoluteJointDef.bodyB = right_arm_lower;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  joint3 = (b2RevoluteJoint*) m_world->CreateJoint( &revoluteJointDef );


             


                  //----HAND LEFT-----
                  bodyDef.position.Set(28,30);
                  
                  polygonShape.SetAsBox(2,0.5, b2Vec2(0,0), 0 );
                  right_hand_left = m_world->CreateBody( &bodyDef );
                  right_hand_left->CreateFixture( &fixtureDef );
                  right_hand_left->SetGravityScale(0);

                  //----HAND JOINT LEFT-----
                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-2.8,0);
                  revoluteJointDef.bodyA = right_arm_lower;
                  revoluteJointDef.bodyB = right_hand_left;
                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  -45 * DEGTORAD;
                  m_world->CreateJoint( &revoluteJointDef );

                  //----FINGER-left-----
                  bodyDef.position.Set(34,30);
                  polygonShape.SetAsBox(2, 0.5 );
                  right_finger_left = m_world->CreateBody( &bodyDef );
                  right_finger_left->CreateFixture( &fixtureDef );
                  right_finger_left->SetGravityScale(0);
                  revoluteJointDef.localAnchorA.Set(2.5,0);
                  revoluteJointDef.localAnchorB.Set(-2.5,0);
                  revoluteJointDef.bodyA = right_hand_left;
                  revoluteJointDef.bodyB = right_finger_left;

                  revoluteJointDef.lowerAngle =  0 * DEGTORAD;
                  revoluteJointDef.upperAngle =   80* DEGTORAD;
        
                   m_world->CreateJoint( &revoluteJointDef );

                  //----HAND RIGHT-----
                  bodyDef.position.Set(28,30);
                  polygonShape.SetAsBox(2,0.5, b2Vec2(+0,0), 0);
                  right_hand_right = m_world->CreateBody( &bodyDef );
                  right_hand_right->CreateFixture( &fixtureDef );
                  right_hand_right->SetGravityScale(0);
                  m_world->CreateJoint( &revoluteJointDef );


                  // ----------HAND JOINT RIGHT--------------
                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-2.8,0);
                  revoluteJointDef.bodyA = right_arm_lower;
                  revoluteJointDef.bodyB = right_hand_right;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = 45 * DEGTORAD;
                  revoluteJointDef.upperAngle = 90 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //-----FINGER Right------------------
                  bodyDef.position.Set(34,30);
                  polygonShape.SetAsBox(2, 0.5);
                  right_finger_right = m_world->CreateBody( &bodyDef );
                  right_finger_right->CreateFixture( &fixtureDef );
                  right_finger_right->SetGravityScale(0);
                  revoluteJointDef.localAnchorA.Set(2.5,0);
                  revoluteJointDef.localAnchorB.Set(-2.5,0);
                  revoluteJointDef.bodyA = right_hand_right;
                  revoluteJointDef.bodyB = right_finger_right;

                  revoluteJointDef.lowerAngle =  -80 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );


                 //---------MASSA------------------------

                  bodyDef.position.Set(0,30);
                  bodyDef.type = b2_dynamicBody;
                  polygonShape.SetAsBox(2,2);
                  mass = m_world->CreateBody( &bodyDef );
                  mass->CreateFixture( &fixtureDef );
                  mass->SetGravityScale(0); 




/// pavimento
            bodyDef.type = b2_staticBody;
            bodyDef.position.Set(-5,0); //middle, bott

            b2EdgeShape edgeShape;
            edgeShape.Set( b2Vec2(-18,0), b2Vec2(28,0) ); //ends of the line
            b2Body* staticBody2 = m_world->CreateBody(&bodyDef);
            staticBody2->CreateFixture(&edgeShape, 0); //add a fixture to the body






//////////////////////////////////////////


              i = 0 ;

              }


    	void Keyboard(unsigned char key)
	{
        switch (key)
		{
	
        case 'g':
            torqueOn = !torqueOn;//toggle bool value
	        break;
		
       
       case 'a':
            addAngle = !addAngle;
               break;

       case 'b':
            lessAngle =!lessAngle;
        }
	}

        void Step(Settings* settings)
        {
            //run the default physics and rendering
               Test::Step(settings);
               //i Contatore

                i = i+1;
                c =i/100;

                m_debugDraw.DrawString(5, m_textLine, "contatore : %4.2f",(float) c);



                /// TORQUE PID

		actualAnglejoint1 = joint1->GetJointAngle();
		pid.setError(-(actualAnglejoint1-desiredAngle));
                pid.step( 1 / settings->hz );
		forceVal = pid.getOutput();
      		right_arm_upper->ApplyTorque(forceVal*8500);

                actualAnglejoint2 = joint2->GetJointAngle();
		pid2.setError(-(actualAnglejoint2-desiredAngle2));
                pid2.step( 1 / settings->hz );
		forceVal2 = pid2.getOutput();
      		right_arm_middle->ApplyTorque(forceVal2*2500);




		actualAnglejoint3 = joint3->GetJointAngle();
		pid3.setError(-(actualAnglejoint3-desiredAngle3));
                pid3.step( 1 / settings->hz );
		forceVal3 = pid3.getOutput();
      		right_arm_lower->ApplyTorque(forceVal3*1500);
               

		float value =joint1->GetJointAngle() ;
		//     STAMPA SULLO SCHERMO

	   m_textLine += 25;
           m_debugDraw.DrawString(5, m_textLine, "errore sul primo joint : %4.2f",(float) -                (actualAnglejoint1-desiredAngle));
           m_textLine += 25;
           m_debugDraw.DrawString(5, m_textLine, "errore sul secondo joint : %4.2f",(float) -(actualAnglejoint2-desiredAngle2));
           m_textLine += 25;
           m_debugDraw.DrawString(5, m_textLine, "errore sul terzo joint : %4.2f",(float) -(actualAnglejoint3-desiredAngle3));
           m_textLine += 25;
           m_debugDraw.DrawString(5, m_textLine, "output : %4.2f",(float) forceVal);
           m_textLine += 25;
           m_debugDraw.DrawString(5, m_textLine, "output2 : %4.2f",(float) forceVal2);
           m_textLine += 25;
           m_debugDraw.DrawString(5, m_textLine, "output3 : %4.2f",(float) forceVal3);
           m_textLine += 25;

     //KeySwitch :

                if (torqueOn){
                right_hand_left->ApplyTorque(10000);
                right_hand_right->ApplyTorque( -10000 );
                right_finger_left->ApplyTorque(10000);
                right_finger_right->ApplyTorque(-10000 );
                 }
                else {
                right_hand_left->ApplyTorque(-1000);
                right_hand_right->ApplyTorque(1000 );
                right_finger_left->ApplyTorque(-1000);
                right_finger_right->ApplyTorque(1000 );
                      }
 
                if(addAngle){
                desiredAngle -= 10*DEGTORAD ;
                desiredAngle2 -= 10*DEGTORAD ;
                desiredAngle3 -= 10*DEGTORAD ;
                addAngle = false;
                }
                else{ 
                }

                if(lessAngle){
                desiredAngle += 10*DEGTORAD ;
                desiredAngle2 += 10*DEGTORAD ;
                desiredAngle3 += 10*DEGTORAD ;
                lessAngle = false;
                }
                else{ 
                }

        }

        static Test* Create()
        {
            return new roboticarm;
        }
    };

  #endif
