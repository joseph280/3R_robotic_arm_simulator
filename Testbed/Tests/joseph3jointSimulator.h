#ifndef JOSEPHJOINT_H
#define JOSEPHJOINT_H


#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f


#include "PIDController.h"



  class josephjoint : 
                         
                       public Test


  {


// OBJ 

b2Body* body_center;
b2Body* right_arm_upper;
b2Body* right_arm_lower;
b2Body* right_arm_middle;
//b2Body* left_arm_upper;
//b2Body* left_arm_lower;
b2Body* right_hand_right;
b2Body* right_hand_left;
//b2Body* left_hand_right;
//b2Body* left_hand_left;
b2RevoluteJoint* joint1;
b2RevoluteJoint* joint2;
b2RevoluteJoint* joint3;

// PID controller
PIDController pid;
PIDController pid2;
PIDController pid3;

// joint 
float actualAnglejoint1;
float actualAnglejoint2;
float actualAnglejoint3;
//torque
float forceVal;
float forceVal2;
float forceVal3;
//angoli
float desiredAngle;
float desiredAngle2;
float desiredAngle3;
// variabili contatore
float i;
float c;
bool torqueOn =false;


public:



   josephjoint() {

                  pid = PIDController();
                  pid.setGains(3,35,1);
                  pid2 = PIDController();
                  pid2.setGains(3,35,1);
                  pid3 = PIDController();
                  pid3.setGains(3,35,1);

                  desiredAngle = -10*DEGTORAD;
                  desiredAngle2 = -10*DEGTORAD;
                  desiredAngle3 = -10*DEGTORAD;



              //COMMON DEFINITIONS

              //Variables
                  b2BodyDef bodyDef;
                  b2FixtureDef fixtureDef;
                  b2PolygonShape polygonShape;
                  b2RevoluteJointDef revoluteJointDef;




              //Algorithm



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
                  polygonShape.SetAsBox(3,1);

                  right_arm_upper = m_world->CreateBody( &bodyDef );
                  right_arm_upper->CreateFixture( &fixtureDef );
                  //right_arm_upper->SetGravityScale(1);

                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-4,0);
                  revoluteJointDef.bodyA = body_center;
                  revoluteJointDef.bodyB = right_arm_upper;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  joint1 = (b2RevoluteJoint*) m_world->CreateJoint( &revoluteJointDef );

             

                // Revolute Joint  


                  bodyDef.position.Set(16,30);
                  polygonShape.SetAsBox(3,1);

                  right_arm_middle = m_world->CreateBody( &bodyDef );
                  right_arm_middle->CreateFixture( &fixtureDef );
                 // right_arm_middle->SetGravityScale(1);

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
                  polygonShape.SetAsBox(3,1);

                  right_arm_lower = m_world->CreateBody( &bodyDef );
                  right_arm_lower->CreateFixture( &fixtureDef );
         //       right_arm_lower->SetGravityScale(1);
 
                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-4,0);
                  revoluteJointDef.bodyA = right_arm_middle;
                  revoluteJointDef.bodyB = right_arm_lower;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  joint3 = (b2RevoluteJoint*) m_world->CreateJoint( &revoluteJointDef );
//Mano da rifare


                  //----RIGHT HAND LEFT-----
                  bodyDef.position.Set(28,30);
                  polygonShape.SetAsBox(2,0.5, b2Vec2(0,-5), 0 );
                  right_hand_left = m_world->CreateBody( &bodyDef );
                  right_hand_left->CreateFixture( &fixtureDef );
                  polygonShape.SetAsBox(2, 0.5, b2Vec2(-2,-3), 90*DEGTORAD );
                  right_hand_left->CreateFixture( &fixtureDef );
                  right_hand_left->SetGravityScale(1);

                  //----RIGHT HAND JOINT LEFT-----
                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(0,0);
                  revoluteJointDef.bodyA = right_arm_lower;
                  revoluteJointDef.bodyB = right_hand_left;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = 0 * DEGTORAD;
                  revoluteJointDef.upperAngle =  90 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //----RIGHT HAND RIGHT-----
                  bodyDef.position.Set(28,30);
                  polygonShape.SetAsBox(2,0.5, b2Vec2(0,5), 0 );
                  right_hand_right = m_world->CreateBody( &bodyDef );
                  right_hand_right->CreateFixture( &fixtureDef );
                  polygonShape.SetAsBox(2, 0.5, b2Vec2(-2,3), -90*DEGTORAD );

                  right_hand_right->CreateFixture( &fixtureDef );
                  right_hand_right->SetGravityScale(1);

                  //----RIGHT HAND JOINT RIGHT-----
                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(0,0);
                  revoluteJointDef.bodyA = right_arm_lower;
                  revoluteJointDef.bodyB = right_hand_right;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );


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
      		right_arm_upper->ApplyTorque(forceVal*10000);

                actualAnglejoint2 = joint2->GetJointAngle();
		pid2.setError(-(actualAnglejoint2-desiredAngle2));
                pid2.step( 1 / settings->hz );
		forceVal2 = pid2.getOutput();
      		right_arm_middle->ApplyTorque(forceVal2*10000);




		actualAnglejoint3 = joint3->GetJointAngle();
		pid3.setError(-(actualAnglejoint3-desiredAngle3));
                pid3.step( 1 / settings->hz );
		forceVal3 = pid3.getOutput();
      		right_arm_lower->ApplyTorque(forceVal3*10000);
               

		float value =joint1->GetJointAngle() ;
		//     STAMPA SULLO SCHERMO

	   m_textLine += 25;
           m_debugDraw.DrawString(5, m_textLine, "errore sul primo joint : %4.2f",(float) -(actualAnglejoint1-desiredAngle));
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

                 }

     else {

                right_hand_left->ApplyTorque(-10000);
                right_hand_right->ApplyTorque(10000 );

                      }



        }

        static Test* Create()
        {
            return new josephjoint;
        }
    };

  #endif
