#ifndef JIOO_H
#define JIOO_H


#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f


  class Jioo : public Test
  {
b2Body* dynamicBody;

        public:
              Jioo() {

//body and fixture defs are common to all chain links
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position.Set(5,10);
  b2FixtureDef fixtureDef;
  fixtureDef.density = 1;
  b2PolygonShape polygonShape;
  polygonShape.SetAsBox(4,1.0);
  fixtureDef.shape = &polygonShape;

  //create first link
  b2Body* link = m_world->CreateBody( &bodyDef );
  link->CreateFixture( &fixtureDef );

//set up the common properties of the joint before entering the loop
  b2RevoluteJointDef revoluteJointDef;
  revoluteJointDef.localAnchorA.Set( 3.3,0);
  revoluteJointDef.localAnchorB.Set(-3.3,0);
//
  //use same definitions to create multiple bodies
//  for (int i = 0; i < 1; i++)

      b2Body* newLink = m_world->CreateBody( &bodyDef );

//


      newLink->CreateFixture( &fixtureDef );
      //newLink->CreateFixture(&fixtureDef2);
      //...joint creation will go here...
 {
            b2BodyDef bodyDef1;

            bodyDef1.type = b2_dynamicBody;
            b2FixtureDef fixtureDef2;
            fixtureDef2.density = 1;
            fixtureDef2.shape = &polygonShape;

            polygonShape.SetAsBox(4, 0.5);
            bodyDef1.position.Set(5,40);

            b2Body* dynamicBody5 = m_world->CreateBody(&bodyDef1);
            dynamicBody5->CreateFixture(&fixtureDef2);
            //link->CreateFixture(&fixtureDef2);

}








//inside the loop, only need to change the bodies to be joined
      revoluteJointDef.bodyA = link;
      revoluteJointDef.bodyB = newLink;
      m_world->CreateJoint( &revoluteJointDef );
      link = newLink;//prepare for next iteration

//// staticBody2
            b2PolygonShape boxShape;
            polygonShape.SetAsBox(2,2);
            bodyDef.type = b2_staticBody; //this will be a static body
            bodyDef.position.Set(0,30); //slightly lower position
            b2Body* staticBody = m_world->CreateBody(&bodyDef); //add body to world
            staticBody->CreateFixture(&fixtureDef); //add fixture to body

  //another revolute joint
  revoluteJointDef.bodyA = link;//
  revoluteJointDef.bodyB = staticBody;
  revoluteJointDef.localAnchorA.Set(5.0,0);//
  revoluteJointDef.localAnchorB.Set(0,0);//
  m_world->CreateJoint( &revoluteJointDef );

//
            bodyDef.type = b2_dynamicBody;
            polygonShape.SetAsBox(1, 1);
            bodyDef.position.Set(5,40);
            b2Body* dynamicBody4 = m_world->CreateBody(&bodyDef);
            dynamicBody4->CreateFixture(&fixtureDef);


//

/// pavimento
            bodyDef.type = b2_staticBody;
            bodyDef.position.Set(-5,0); //middle, bott

            b2EdgeShape edgeShape;
            edgeShape.Set( b2Vec2(-18,0), b2Vec2(28,0) ); //ends of the line
            b2Body* staticBody2 = m_world->CreateBody(&bodyDef);
            staticBody2->CreateFixture(&edgeShape, 0); //add a fixture to the body


   } //do nothing, no scene yet

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
            return new Jioo;
        }

    };

  #endif
