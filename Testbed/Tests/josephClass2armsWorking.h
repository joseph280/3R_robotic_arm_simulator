#ifndef JOSEPHCLASS_H
#define JOSEPHCLASS_H


#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f


  class josephTest : public Test
  {
b2Body* body_center;
b2Body* right_arm_upper;
b2Body* right_arm_lower;
b2Body* left_arm_upper;
b2Body* left_arm_lower;
b2Body* right_hand_right;
b2Body* right_hand_left;
b2Body* left_hand_right;
b2Body* left_hand_left;

bool torqueOn =false;


        public:
              josephTest() {


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
                  bodyDef.position.Set(0,10);

                  //----CENTRAL BODY-----
                  body_center = m_world->CreateBody( &bodyDef );
                  body_center->CreateFixture( &fixtureDef );
                  body_center->SetGravityScale(0);

                                    bodyDef.type = b2_dynamicBody;


                  //----RIGHT ARM UPPER-----
                  bodyDef.position.Set(10,10);
                  polygonShape.SetAsBox(3,1);

                  right_arm_upper = m_world->CreateBody( &bodyDef );
                  right_arm_upper->CreateFixture( &fixtureDef );
                  right_arm_upper->SetGravityScale(0);

                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-4,0);
                  revoluteJointDef.bodyA = body_center;
                  revoluteJointDef.bodyB = right_arm_upper;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = 0 * DEGTORAD;
                  revoluteJointDef.upperAngle =  90 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //----LEFT ARM UPPER-----
                  bodyDef.position.Set(-10,10);
                  polygonShape.SetAsBox(3,1);

                  left_arm_upper = m_world->CreateBody( &bodyDef );
                  left_arm_upper->CreateFixture( &fixtureDef );
                  left_arm_upper->SetGravityScale(0);

                  revoluteJointDef.localAnchorA.Set(-4,0);
                  revoluteJointDef.localAnchorB.Set(4,0);
                  revoluteJointDef.bodyA = body_center;
                  revoluteJointDef.bodyB = left_arm_upper;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //----RIGHT ARM LOWER-----
                  bodyDef.position.Set(16,10);
                  polygonShape.SetAsBox(3,1);

                  right_arm_lower = m_world->CreateBody( &bodyDef );
                  right_arm_lower->CreateFixture( &fixtureDef );
                  right_arm_lower->SetGravityScale(0);

                  revoluteJointDef.localAnchorA.Set(4,0);
                  revoluteJointDef.localAnchorB.Set(-4,0);
                  revoluteJointDef.bodyA = right_arm_upper;
                  revoluteJointDef.bodyB = right_arm_lower;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = 0 * DEGTORAD;
                  revoluteJointDef.upperAngle =  90 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //----LEFT ARM LOWER-----
                  bodyDef.position.Set(-16,10);
                  polygonShape.SetAsBox(3,1);

                  left_arm_lower = m_world->CreateBody( &bodyDef );
                  left_arm_lower->CreateFixture( &fixtureDef );
                  left_arm_lower->SetGravityScale(0);

                  revoluteJointDef.localAnchorA.Set(-4,0);
                  revoluteJointDef.localAnchorB.Set(4,0);
                  revoluteJointDef.bodyA = left_arm_upper;
                  revoluteJointDef.bodyB = left_arm_lower;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //----LEFT HAND LEFT-----
                  bodyDef.position.Set(-22,10);
                  polygonShape.SetAsBox(2,0.5, b2Vec2(0,-5), 0 );
                  left_hand_left = m_world->CreateBody( &bodyDef );
                  left_hand_left->CreateFixture( &fixtureDef );
                  polygonShape.SetAsBox(2, 0.5, b2Vec2(2,-3), 90*DEGTORAD );
                  left_hand_left->CreateFixture( &fixtureDef );
                  left_hand_left->SetGravityScale(0);

                  //----LEFT HAND JOINT LEFT-----
                  revoluteJointDef.localAnchorA.Set(-4,0);
                  revoluteJointDef.localAnchorB.Set(0,0);
                  revoluteJointDef.bodyA = left_arm_lower;
                  revoluteJointDef.bodyB = left_hand_left;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = -90 * DEGTORAD;
                  revoluteJointDef.upperAngle =  0 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //----LEFT HAND RIGHT-----
                  bodyDef.position.Set(-22,10);
                  polygonShape.SetAsBox(2,0.5, b2Vec2(0,5), 0 );
                  left_hand_right = m_world->CreateBody( &bodyDef );
                  left_hand_right->CreateFixture( &fixtureDef );
                  polygonShape.SetAsBox(2, 0.5, b2Vec2(2,3), -90*DEGTORAD );
                  left_hand_right->CreateFixture( &fixtureDef );
                  left_hand_right->SetGravityScale(0);

                  //----LEFT HAND JOINT RIGHT-----
                  revoluteJointDef.localAnchorA.Set(-4,0);
                  revoluteJointDef.localAnchorB.Set(0,0);
                  revoluteJointDef.bodyA = left_arm_lower;
                  revoluteJointDef.bodyB = left_hand_right;

                  revoluteJointDef.collideConnected = true;
                  revoluteJointDef.enableLimit = true;
                  revoluteJointDef.lowerAngle = 0 * DEGTORAD;
                  revoluteJointDef.upperAngle =  90 * DEGTORAD;

                  m_world->CreateJoint( &revoluteJointDef );

                  //----RIGHT HAND LEFT-----
                  bodyDef.position.Set(22,10);
                  polygonShape.SetAsBox(2,0.5, b2Vec2(0,-5), 0 );
                  right_hand_left = m_world->CreateBody( &bodyDef );
                  right_hand_left->CreateFixture( &fixtureDef );
                  polygonShape.SetAsBox(2, 0.5, b2Vec2(-2,-3), 90*DEGTORAD );
                  right_hand_left->CreateFixture( &fixtureDef );
                  right_hand_left->SetGravityScale(0);

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
                  bodyDef.position.Set(22,10);
                  polygonShape.SetAsBox(2,0.5, b2Vec2(0,5), 0 );
                  right_hand_right = m_world->CreateBody( &bodyDef );
                  right_hand_right->CreateFixture( &fixtureDef );
                  polygonShape.SetAsBox(2, 0.5, b2Vec2(-2,3), -90*DEGTORAD );
                  right_hand_right->CreateFixture( &fixtureDef );
                  right_hand_right->SetGravityScale(0);

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

              }
    	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'w':

         body_center->ApplyForce( b2Vec2(0,1000), body_center->GetWorldCenter() );
			break;

		case 's':

         body_center->ApplyForce( b2Vec2(0,-1000), body_center->GetWorldCenter() );
			break;

			case 'd':

         body_center->ApplyForce( b2Vec2(1000,0), body_center->GetWorldCenter() );
			break;

		case 'a':
         body_center->ApplyForce( b2Vec2(-1000,0), body_center->GetWorldCenter() );
			break;

        case 'g':
            torqueOn = !torqueOn;//toggle bool value
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



      if (torqueOn){
         left_hand_left->ApplyTorque(10000);
         left_hand_right->ApplyTorque( -10000 );
      } else {
         left_hand_left->ApplyTorque(-10000);
         left_hand_right->ApplyTorque(10000 );
      }


        }

        static Test* Create()
        {
            return new josephTest;
        }
    };

  #endif
