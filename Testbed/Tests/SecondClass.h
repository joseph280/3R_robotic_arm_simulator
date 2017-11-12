#ifndef SECONDCLASS_H
#define SECONDCLASS_H


class SecondClass : public Test
{

//in the class itself, not inside the constructor!
b2Body* dynamicBody;

 public:
        SecondClass() {

            //DYNAMIC
            b2BodyDef myBodyDef;
            myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
            myBodyDef.position.Set(0, 20); //set the starting position
            myBodyDef.angle = 0; //set the starting angle

            //Creo el body
            dynamicBody = m_world->CreateBody(&myBodyDef);

              //prepare a shape definition
            b2PolygonShape polygonShape;
            b2FixtureDef myFixtureDef;
            myFixtureDef.shape = &polygonShape;
            myFixtureDef.density = 1;



             //add four square shaped fixtures around the body center
            for ( int i = 0; i < 4; i++) {
              b2Vec2 pos( sinf(i*90*DEGTORAD), cosf(i*90*DEGTORAD) ); //radial placement
              polygonShape.SetAsBox(1, 1, pos, 0 ); //a 2x2 rectangle
              myFixtureDef.friction =  i/4.0;
              dynamicBody->CreateFixture(&myFixtureDef); //add a fixture to the body
            }

            //Adding fixtures
            b2PolygonShape boxShape;
            boxShape.SetAsBox(1,1);

            b2FixtureDef boxFixtureDef;
            boxFixtureDef.shape = &boxShape;
            boxFixtureDef.density = 100;
            dynamicBody->CreateFixture(&boxFixtureDef);

            #define DEGTORAD 0.0174532925199432957f
            #define RADTODEG 57.295779513082320876f

            dynamicBody->SetTransform( b2Vec2( 10, 20 ), 45 * DEGTORAD ); //45 degrees counter-clockwise
            dynamicBody->SetLinearVelocity( b2Vec2( -5, 5 ) ); //moving up and left 5 units per second
            dynamicBody->SetAngularVelocity( -90 * DEGTORAD ); //90 degrees per second clockwise

            //CIRCLE
            myBodyDef.position.Set(-10, 20); //a little to the left

            b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
            b2CircleShape circleShape;
            circleShape.m_p.Set(0, 0); //position, relative to body position
            circleShape.m_radius = 1; //radius

            b2FixtureDef myFixtureDef2;
            myFixtureDef2.shape = &circleShape; //this is a pointer to the shape above
            dynamicBody1->CreateFixture(&myFixtureDef2); //add a fixture to the body

            //POLYGON
            //set each vertex of polygon in an array
            b2Vec2 vertices[5];
           /* vertices[0].Set(-1,  2);
            vertices[1].Set(-1,  0);
            vertices[2].Set( 0, -3);
            vertices[3].Set( 1,  0);
            vertices[4].Set( 1,  1); */
             //for the custom polygon, add 10 to each x-coord
            vertices[0].Set(-1 +3,  2);
            vertices[1].Set(-1 +3,  0);
            vertices[2].Set( 0 +3, -3);
            vertices[3].Set( 1 +3,  0);
            vertices[4].Set( 1 +3,  1);

             b2Vec2 vertices2[5];
           /* vertices[0].Set(-1,  2);
            vertices[1].Set(-1,  0);
            vertices[2].Set( 0, -3);
            vertices[3].Set( 1,  0);
            vertices[4].Set( 1,  1); */
             //for the custom polygon, add 10 to each x-coord
            vertices2[0].Set(-1 -3,  2);
            vertices2[1].Set(-1 -3,  0);
            vertices2[2].Set( 0 -3, -3);
            vertices2[3].Set( 1 -3,  0);
            vertices2[4].Set( 1 -3,  1);

            polygonShape.Set(vertices, 5); //pass array to the shape

            b2PolygonShape polygonShape2;
            polygonShape2.Set(vertices2, 5); //pass array to the shape

            myFixtureDef.shape = &polygonShape; //change the shape of the fixture
            b2FixtureDef myFixtureDef5;
            myFixtureDef5.shape = &polygonShape2; //change the shape of the fixture
            myBodyDef.position.Set(0, 20); //in the middle
            b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
            dynamicBody2->CreateFixture(&myFixtureDef); //add a fixture to the body

            //MIXED BODY
            myBodyDef.position.Set(5,5); //a bit to the right
            b2Body* dynamicBody4 = m_world->CreateBody(&myBodyDef); //add body to world
            dynamicBody4->CreateFixture(&boxFixtureDef); //add fixture to body
            dynamicBody4->CreateFixture(&myFixtureDef); //add fixture to body
            dynamicBody4->CreateFixture(&myFixtureDef2); //add fixture to body
            dynamicBody4->CreateFixture(&myFixtureDef5); //add fixture to body

            //set up the definition for a xxx joint
 // b2RevoluteJoint jointDef = b2RevoluteJointDef();

 // jointDef.Revolute = ...;

  //create the joint
//  b2RevoluteJoint* joint = (b2RevoluteJoint*)world->CreateJoint( &jointDef );

  // jointDef.dynamicBody4 = upperLegBody;
 // jointDef.polygonShape2 = lowerLegBody;
 // jointDef.collideConnected = false;

            //RECTANGULAR
            polygonShape.SetAsBox(2, 1); //a 4x2 rectangle
            myBodyDef.position.Set(10,20); //a bit to the right

            b2Body* dynamicBody3 = m_world->CreateBody(&myBodyDef);
            dynamicBody3->CreateFixture(&myFixtureDef); //add a fixture to the body

            //STATIC
            myBodyDef.type = b2_staticBody; //this will be a static body
            myBodyDef.position.Set(0, 10); //slightly lower position
            b2Body* staticBody = m_world->CreateBody(&myBodyDef); //add body to world
            staticBody->CreateFixture(&boxFixtureDef); //add fixture to body

            //A WALL
            myBodyDef.position.Set(-5,0); //middle, bottom

            b2EdgeShape edgeShape;
            edgeShape.Set( b2Vec2(-15,0), b2Vec2(15,3) ); //ends of the line
            b2Body* staticBody2 = m_world->CreateBody(&myBodyDef);
            staticBody2->CreateFixture(&edgeShape, 0); //add a fixture to the body

            //KINEMATIC
            myBodyDef.type = b2_kinematicBody; //this will be a kinematic body
            myBodyDef.position.Set(-18, 11); // start from left side, slightly above the static body
            b2Body* kinematicBody = m_world->CreateBody(&myBodyDef); //add body to world
            kinematicBody->CreateFixture(&boxFixtureDef); //add fixture to body

            kinematicBody->SetLinearVelocity( b2Vec2( 1, 0 ) ); //move right 1 unit per second
            kinematicBody->SetAngularVelocity( 360 * DEGTORAD ); //1 turn per second counter-clockwise

        } //do nothing, no scene yet

    void Step(Settings* settings)
    {
        //run the default physics and rendering
        Test::Step(settings);

        /*b2Vec2 pos = dynamicBody->GetPosition();
        float angle = dynamicBody->GetAngle();
        b2Vec2 vel = dynamicBody->GetLinearVelocity();
        float angularVel = dynamicBody->GetAngularVelocity();
        m_debugDraw.DrawString(5, m_textLine,
            "Position:%.3f,%.3f Angle:%.3f", pos.x, pos.y, angle * RADTODEG);
        m_textLine += 15;
        m_debugDraw.DrawString(5, m_textLine,
            "Velocity:%.3f,%.3f Angular velocity:%.3f", vel.x, vel.y, angularVel * RADTODEG);
        m_textLine += 15; */

        //A test
        //dynamicBody->SetTransform( dynamicBody->GetPosition(), dynamicBody->GetAngle() );
        for ( b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
        {
            b2Vec2 pos = b->GetPosition();
            float angle = b->GetAngle();
            b2Vec2 vel = b->GetLinearVelocity();
            float angularVel = b->GetAngularVelocity();
            m_debugDraw.DrawString(5, m_textLine,
                    "Position:%.3f,%.3f Angle:%.3f", pos.x, pos.y, angle * RADTODEG);
            m_textLine += 15;
            m_debugDraw.DrawString(5, m_textLine,
                    "Velocity:%.3f,%.3f Angular velocity:%.3f", vel.x, vel.y, angularVel * RADTODEG);
            m_textLine += 15;
        }

  //DELETES A BODY
   // m_world->DestroyBody(dynamicBody);


        //show some text in the main screen
        m_debugDraw.DrawString(5, m_textLine, "Now we have a testClass test");
        m_textLine += 15;
    }

    static Test* Create()
    {
        return new SecondClass;
    }
};

#endif // TESTCLASS_H

