#ifndef JIO_H
#define JIO_H

  class Jio : public Test
  {
b2Body* dynamicBody;
b2Body* dynamicBody5;
b2Body* dynamicBody4;
b2Body* dynamicBody3;
b2Body* dynamicBody6;
b2Body* dynamicBody7;

        public:
              Jio() {


                           b2BodyDef myBodyDef;
                           myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
                           myBodyDef.position.Set(0, 25); //set the starting position
                           myBodyDef.angle = 0; //set the starting angle

                           dynamicBody = m_world->CreateBody(&myBodyDef);
                           b2PolygonShape boxShape;
                           boxShape.SetAsBox(1,3);

//2
         b2FixtureDef boxFixtureDef;
         boxFixtureDef.shape = &boxShape;
         boxFixtureDef.density = 1;
        // boxFixtureDef.density = 1;
         dynamicBody->CreateFixture(&boxFixtureDef);

            #define DEGTORAD 0.0174532925199432957f
            #define RADTODEG 57.295779513082320876f
            dynamicBody->SetLinearVelocity( b2Vec2( -5, 5 ) ); //moving up and left 5 units per second
            dynamicBody->SetAngularVelocity( -90 * DEGTORAD ); //90 degrees per second clockwise

/*
            b2PolygonShape polygonShape;/// va prima della definizione della fixture
            b2FixtureDef myFixtureDef;


            dynamicBody1->CreateFixture(&myFixtureDef); //add a fixture to the body1

            myFixtureDef.shape = &polygonShape; //change

            polygonShape.SetAsBox(2, 1); //a 4x2 rectangle
            myBodyDef.position.Set(10,20); //a bit to the right
            b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);

            dynamicBody1->SetLinearVelocity( b2Vec2( -5, 5 ) ); //moving up and left 5 units per second
            dynamicBody1->SetAngularVelocity( -90 * DEGTORAD );

*/

            b2PolygonShape polygonShape;//added

            b2FixtureDef myFixtureDef;
            myFixtureDef.shape = &polygonShape; //change the shape of the fixture

            //RECTANGULAR
            polygonShape.SetAsBox(1, 3); //a 4x2 rectangle
            myBodyDef.position.Set(1,20); //a bit to the right
            myBodyDef.angle = 0* DEGTORAD;
           // myBodyDef.angle = 30* DEGTORAD;

            dynamicBody3 = m_world->CreateBody(&myBodyDef);
            dynamicBody3->CreateFixture(&myFixtureDef);
            polygonShape.SetAsBox(1, 1);
            myBodyDef.position.Set(0,31);
            dynamicBody4 = m_world->CreateBody(&myBodyDef);
            dynamicBody4->CreateFixture(&myFixtureDef);

            polygonShape.SetAsBox(1, 3); //a 4x2 rectangle
            myBodyDef.position.Set(5,5); //a bit to the right
            myBodyDef.angle = 0* DEGTORAD;
            dynamicBody6 = m_world->CreateBody(&myBodyDef);
            dynamicBody6->CreateFixture(&myFixtureDef);
            polygonShape.SetAsBox(1, 4);
            myBodyDef.position.Set(-5,5);
            dynamicBody7 = m_world->CreateBody(&myBodyDef);
            dynamicBody7->CreateFixture(&myFixtureDef);

//CENTER of the robot
            boxShape.SetAsBox(2,2);
            myBodyDef.type = b2_dynamicBody; //this will be a static body
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0,30); //slightly lower position
            dynamicBody5 = m_world->CreateBody(&myBodyDef); //add body to world
            dynamicBody5->CreateFixture(&boxFixtureDef); //add fixture to body
            myBodyDef.position.Set(-5,0); //middle, bottom


       //     dynamicBody5->SetGravityScale(1);
       //     dynamicBody3->SetGravityScale(1);
       //     dynamicBody->SetGravityScale(1);



       /// REVOLUtE 1

              b2RevoluteJointDef revoluteJointDef;
              revoluteJointDef.bodyB = dynamicBody5;
              revoluteJointDef.bodyA = dynamicBody;
              revoluteJointDef.collideConnected = false;

              revoluteJointDef.enableLimit = true;
              revoluteJointDef.lowerAngle = -90 * DEGTORAD;
              revoluteJointDef.upperAngle =  90 * DEGTORAD;


              //alter joint limits
            void EnableLimit(bool enabled);
              void SetLimits( float lower, float upper );

              //query joint limits
              bool IsLimitEnabled();
              float GetLowerLimit();
              float GetUpperLimit();


              revoluteJointDef.localAnchorA.Set(0,8);
              revoluteJointDef.localAnchorB.Set(0,-1);
         //    revoluteJointDef.localAnchorA.Set(0,8);
          //    revoluteJointDef.localAnchorB.Set(0,1);
              revoluteJointDef.referenceAngle = 0;
              revoluteJointDef.enableMotor = true;
              revoluteJointDef.maxMotorTorque = 20;
              revoluteJointDef.motorSpeed = 360 * DEGTORAD;

              //alter joint motor
  void EnableMotor(bool enabled);
  void SetMotorSpeed(float speed);
  void SetMaxMotorTorque(float torque);

  //query joint motor
  bool IsMotorEnabled();
  float GetMotorSpeed();
  float GetMotorTorque();
  b2Joint* joint = m_world->CreateJoint( &revoluteJointDef );

//REVOLUTE2
/*
b2RevoluteJointDef revoluteJointDef1;
              //  b2RevoluteJointDef revoluteJointDef;
                revoluteJointDef1.bodyA = dynamicBody;
                revoluteJointDef1.bodyB = dynamicBody3;
                revoluteJointDef1.collideConnected = false;

                revoluteJointDef1.enableLimit = true;
                revoluteJointDef1.lowerAngle = -90 * DEGTORAD;
                revoluteJointDef1.upperAngle =  90 * DEGTORAD;

                    //alter joint limits
            void EnableLimit(bool enabled);
              void SetLimits( float lower, float upper );

              //query joint limits
              bool IsLimitEnabled();
              float GetLowerLimit();
              float GetUpperLimit();


                //alter joint limits
              void EnableLimit(bool enabled);
                void SetLimits( float lower, float upper );

                //query joint limits
                bool IsLimitEnabled();
                float GetLowerLimit();
                float GetUpperLimit();




                revoluteJointDef1.localAnchorA.Set(0,-7);
                revoluteJointDef1.localAnchorB.Set(0,6);
             //   revoluteJointDef1.localAnchorA.Set(0,-10);
             //   revoluteJointDef1.localAnchorB.Set(0,1);
                revoluteJointDef1.referenceAngle = 0;
                revoluteJointDef1.enableMotor = true;
                revoluteJointDef1.maxMotorTorque = 20;
                revoluteJointDef1.motorSpeed = 360 * DEGTORAD;

                //alter joint motor
    void EnableMotor(bool enabled);
    void SetMotorSpeed(float speed);
    void SetMaxMotorTorque(float torque);

    //query joint motor
    bool IsMotorEnabled();
    float GetMotorSpeed();
    float GetMotorTorque();
    b2Joint* joint1 = m_world->CreateJoint( &revoluteJointDef1 );
/*
//// staticBody2
boxShape.SetAsBox(3,3);
myBodyDef.type = b2_staticBody; //this will be a static body
myBodyDef.position.Set(0,30); //slightly lower position
b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
staticBody->CreateFixture(&boxFixtureDef); //add fixture to body
myBodyDef.position.Set(-5,0); //middle, bottom

*/




       /// REVOLUtE 3

              b2RevoluteJointDef revoluteJointDef2;
              revoluteJointDef2.bodyB = dynamicBody5;
              revoluteJointDef2.bodyA = dynamicBody6;
              revoluteJointDef2.collideConnected = false;

              revoluteJointDef2.enableLimit = true;
              revoluteJointDef2.lowerAngle = -90 * DEGTORAD;
              revoluteJointDef2.upperAngle =  90 * DEGTORAD;


              //alter joint limits
            void EnableLimit(bool enabled);
              void SetLimits( float lower, float upper );

              //query joint limits
              bool IsLimitEnabled();
              float GetLowerLimit();
              float GetUpperLimit();




              revoluteJointDef2.localAnchorA.Set(0,8);
              revoluteJointDef2.localAnchorB.Set(0,0);
              revoluteJointDef2.referenceAngle = 0;
              revoluteJointDef2.enableMotor = true;
              revoluteJointDef2.maxMotorTorque = 20;
              revoluteJointDef2.motorSpeed = 360 * DEGTORAD;

              //alter joint motor
  void EnableMotor(bool enabled);
  void SetMotorSpeed(float speed);
  void SetMaxMotorTorque(float torque);

  //query joint motor
  bool IsMotorEnabled();
  float GetMotorSpeed();
  float GetMotorTorque();
  b2Joint* joint2 = m_world->CreateJoint( &revoluteJointDef2 );



  //REVOLUTE4

b2RevoluteJointDef revoluteJointDef3;
              //  b2RevoluteJointDef revoluteJointDef;
                revoluteJointDef3.bodyA = dynamicBody6;
                revoluteJointDef3.bodyB = dynamicBody7;
                revoluteJointDef3.collideConnected = false;

                revoluteJointDef3.enableLimit = true;
                revoluteJointDef3.lowerAngle = -90 * DEGTORAD;
                revoluteJointDef3.upperAngle =  90 * DEGTORAD;

                    //alter joint limits
            void EnableLimit(bool enabled);
              void SetLimits( float lower, float upper );

              //query joint limits
              bool IsLimitEnabled();
              float GetLowerLimit();
              float GetUpperLimit();


                //alter joint limits
              void EnableLimit(bool enabled);
                void SetLimits( float lower, float upper );

                //query joint limits
                bool IsLimitEnabled();
                float GetLowerLimit();
                float GetUpperLimit();




                revoluteJointDef3.localAnchorA.Set(0,6);
                revoluteJointDef3.localAnchorB.Set(0,4);
                revoluteJointDef3.referenceAngle = 0;
                revoluteJointDef3.enableMotor = true;
                revoluteJointDef3.maxMotorTorque = 20;
                revoluteJointDef3.motorSpeed = 360 * DEGTORAD;

                //alter joint motor
    void EnableMotor(bool enabled);
    void SetMotorSpeed(float speed);
    void SetMaxMotorTorque(float torque);

    //query joint motor
    bool IsMotorEnabled();
    float GetMotorSpeed();
    float GetMotorTorque();
    b2Joint* joint3 = m_world->CreateJoint( &revoluteJointDef3 );







/// pavimento
            myBodyDef.type = b2_staticBody;
            myBodyDef.position.Set(-5,0); //middle, bott

            b2EdgeShape edgeShape;
            edgeShape.Set( b2Vec2(-18,0), b2Vec2(28,0) ); //ends of the line
            b2Body* staticBody2 = m_world->CreateBody(&myBodyDef);
            staticBody2->CreateFixture(&edgeShape, 0); //add a fixture to the body


   } //do nothing, no scene yet

   	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'w':

         dynamicBody5->ApplyForce( b2Vec2(0,1000), dynamicBody5->GetWorldCenter() );
			break;

		case 's':

         dynamicBody5->ApplyForce( b2Vec2(0,-1000), dynamicBody5->GetWorldCenter() );
			break;

			case 'd':

         dynamicBody5->ApplyForce( b2Vec2(1000,0), dynamicBody5->GetWorldCenter() );
			break;

		case 'a':

         dynamicBody5->ApplyForce( b2Vec2(-1000,0), dynamicBody5->GetWorldCenter() );
			break;
		}
	}

        void Step(Settings* settings)
        {
            //run the default physics and rendering
            Test::Step(settings);

            //show some text in the main screen
            m_debugDraw.DrawString(5, m_textLine, "Manipolatore");
            m_textLine += 15;
        }

        static Test* Create()
        {
            return new Jio;
        }
    };

  #endif
