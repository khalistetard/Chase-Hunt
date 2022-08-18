#include "vec.hpp"
#include "draw-triangle-pro.hpp"
#include "raylib-cpp.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cmath>     // std::atan, std::fmod, std::abs
#include <random>    // std::random_device, std::mt19937
#include <variant>   // std::variant, std::visit
#include <iostream>  // std::cout, std::cerr
#include <fstream>   //std::file.open()
#include <chrono>    //for timer
#include <thread>    //to create threads

using Vector = ai::Vector3;  // use x and z in the 2D case

class SteeringOutput
{
public:
  Vector linear_; // These two values each represent an acceleration.
  float angular_; // i.e. they effect changes in velocity (linear and angular).

  SteeringOutput &operator+=(const SteeringOutput &rhs)
  {
    linear_ += rhs.linear_;
    angular_ += rhs.angular_;
    return *this;
  }
  friend SteeringOutput operator*(const float lhs, const SteeringOutput &y) {
    return {lhs*y.linear_, lhs*y.angular_};
  }
};

class Kinematic
{
public:
  Vector position_;
  float orientation_;
  Vector velocity_;
  float rotation_;

  // integration of the linear and angular accelerations
  void update(const SteeringOutput& steering,
              const float maxSpeed,
              float drag,
              const float time) // delta time
  {
    //Newton-Euler 1 simplification:
    position_ += velocity_ * time;
    orientation_ += rotation_ * time;
    orientation_ = std::fmod(orientation_, 2*PI);

    velocity_ += steering.linear_ * time;
    rotation_ += steering.angular_* time;

    post_process(drag, maxSpeed, time);
  }

  void post_process(const float drag, const float maxSpeed, const float time)
  {
    velocity_ *= (1 - drag * time);
    rotation_ *= (1 - drag * time);

    if (velocity_.length() > maxSpeed)
    {
      velocity_.normalise();
      velocity_ *= maxSpeed;
    }
  }

  void Respawn(int w, int h)
  {
      float rand1x = (float)(rand() % (h + ((int)this->position_.x + 51))); //from the collision point raduis 50 to height
      float rand2x = (float)(rand() % (((int)this->position_.x - 50) + 1)); //from 0 to the collision point radius 50

      float rand1z = (float)(rand() % (w + ((int)this->position_.z + 51))); //from the collision point raduis 50 to width
      float rand2z = (float)(rand() % (((int)this->position_.z - 50) + 1)); //from 0 to the collision point radius 50

      int randomx = rand() % 2;
      int randomz = rand() % 2;
      this->position_.x = randomx == 0 ? rand1x : rand2x;
      this->position_.z = randomz == 0 ? rand1z : rand2z;
  }
};

/**
* Class Ground, when the floor is green get bonus score, 
* when it's black, don't touch it you don't want to know what happens 
* By default those grounds are 20 radius of size until they become black grounds they'll then loose 5 radius
* You can change this to higher or lower radius to augment or reduce the difficulty 
* (or for design but we all know the game is not beautifull with the current textures)
*/
class Ground
{
public:
    Ground(const float z, const float x, int size = 20, int size_less = 5)
        : k_{ {x,0,z},(float)0,{0,0,0},0 }, col_{ GREEN }, size_{ size }, size_l{ size_less }, touched{ 0 }, inside_bonus{ 0 }, inside{ 0 } {}

    Kinematic k_;
    raylib::Color col_;
    int size_;
    int size_l;
    int touched;
    int inside_bonus;
    int inside;

    void draw(int screenwidth, int screenheight)
    {
        ai::Vector2 pos{ k_.position_.z, k_.position_.x };
        if (touched == 1)
        {
            ai::DrawCircleLinesV(pos, size_- size_l, col_);
        }
        else
        {
            ai::DrawCircleLinesV(pos, size_, col_);
        }
    }

    //Function for the collision with the ennemy, trap him in but... don't get him too enraged will you ?
    void collision_hunter(Kinematic& ship_k, raylib::Sound& sound, int w, int h, int& score, const float collision_d)
    {
        auto distance = this->k_.position_ - ship_k.position_;
        if (distance.length() <= size_- size_l + collision_d)
        {
            if (touched == 1) // no == operand for col_, no strength to make one (not even sure we can take the RGB)
            {
                score += 1000;
                sound.Play();
                ship_k.Respawn(w, h);
            }
        }
    }
    //Function for the collision with the player, 1st hit grants bonus then the ground goes black, beware !
    Ground collision(Kinematic& ship_k, raylib::Sound& sound1, raylib::Sound& sound2, int w, int h, int& score, int& life, const float collision_d)
    {
        auto distance = this->k_.position_ - ship_k.position_;
        if (distance.length() <= size_ + collision_d)
        {
            if (touched == 0 && inside ==0) // no == operand for col_, no strength to make one (not even sure we can take the RGB)
            {
                score += 1000;
                this->col_ = BLACK; 
                touched = 1;
                inside_bonus = 1;
                inside = 1;
                sound1.Play();
            }
        }
        else
        {
            inside_bonus = 0;
        }

        if (distance.length() <= size_ - size_l + collision_d && inside_bonus == 0)
        {
            if (inside == 0 && touched == 1)
            {
                life--;
                inside = 1;
                ship_k.Respawn(w, h);
                sound2.Play();
            }
        }
        else
        {
            inside = 0;
        }
        
        return *(this);
    }

};

class Ship
{
public:
  Ship(const float z, const float x, const float ori, const raylib::Color col)
    : k_{{x,0,z},ori,{0,0,0},0}, col_{col} { }

  Kinematic k_;
  raylib::Color col_;

  void update(const SteeringOutput& steering,
              const float maxSpeed,
              float drag,
              const float time) { k_.update(steering,maxSpeed,drag,time); }

  void draw(int screenwidth, int screenheight)
  {
    const float w = 10, len = 30; // ship width and length
    const ai::Vector2 l{0, -w}, r{0, w}, nose{len, 0};
    ai::Vector2 pos{k_.position_.z, k_.position_.x};
    float ori = -k_.orientation_ * RAD2DEG; // negate: anticlockwise rot

    // wrap
    pos.x = std::fmod(pos.x, static_cast<float>(screenwidth));
    pos.y = std::fmod(pos.y, static_cast<float>(screenheight));
    pos.x = pos.x < 0 ? pos.x + screenwidth : pos.x;
    pos.y = pos.y < 0 ? pos.y + screenheight : pos.y;

    if (k_.position_.z > screenwidth)
    { k_.position_.z = 0; }
    else if (k_.position_.z < 0)
    { k_.position_.z = screenwidth; }

    if (k_.position_.x > screenheight)
    { k_.position_.x = 0; }
    else if (k_.position_.x < 0)
    { k_.position_.x = screenheight; }


    ai::DrawTrianglePro(pos, l, r, nose, ori, col_);
  }

  //pops a "new" prey when the hunter caught it and play a sound when caught
  void Caught(Ship& ship, int w, int h, int& life_, const float collision_d)
  {
      auto distance = this->k_.position_ - ship.k_.position_;
      if (distance.length() <= collision_d)
      {
          life_--;
          this->k_.Respawn(w, h);
      }

  }
  

};

// Dynamic Seek (page 96)
class Seek
{
public:
  const Kinematic& character_;
  const Kinematic& target_;

  float maxAcceleration_;

  /* // A constructor isn't needed, but adding it will also not hurt
  Seek(Kinematic &c, Kinematic &t, float maxAcceleration)
    : character_{c}, target_{t}, maxAcceleration{maxAcceleration_}
  {
  }*/

  SteeringOutput getSteering() const
  {
    SteeringOutput result;

    result.linear_ = target_.position_ - character_.position_;

    result.linear_.normalise();
    result.linear_ *= maxAcceleration_;

    result.angular_ = 0;
    return result;
  }
};

// Based on Wander
class Onward
{
public:

  const Kinematic& character;

  float maxAcceleration_;
  float maxAngularAcceleration_;
  float headForce_;

  void setHeadForce(const float headForce) { headForce_ = headForce; }

  SteeringOutput getSteering() const
  {
    SteeringOutput result;

    result.linear_ = maxAcceleration_ * ai::asVector(character.orientation_);
    result.angular_ = headForce_ * maxAngularAcceleration_;
    return result;
  };
};

class Align
{
public:
  const Kinematic& character_;
  const Kinematic& target_;

  float maxAngularAcceleration_;
  float maxRotation_;

  float targetRadius_;
  float slowRadius_;

  float timeToTarget_ = 0.1f;

  SteeringOutput getSteering() const
  {
    SteeringOutput result{}; // a zero value

    float rotation = target_.orientation_ - character_.orientation_;

    // mapToRange(rotation); (-pi,pi)
    rotation = std::abs(rotation) > PI ? rotation-2*PI : rotation;
    float rotationSize = std::abs(rotation);
    float targetRotation;

    if (rotationSize < targetRadius_)
      return result;

    if (rotationSize > slowRadius_)
      targetRotation = maxRotation_;
    else
      targetRotation = maxRotation_ * rotationSize / slowRadius_;

    targetRotation *= rotation / rotationSize;

    result.angular_ = targetRotation - character_.rotation_;
    result.angular_ /= timeToTarget_;

    float angularAcceleration = std::abs(result.angular_);
    if (angularAcceleration > maxAngularAcceleration_)
    {
      result.angular_ /= angularAcceleration;
      result.angular_ *= maxAngularAcceleration_;
    }

    result.linear_ = 0;
    return result;
  }
};

class BlendedSteering
{
public:
  class BehaviourAndWeight
  {
  public:
    using SteeringBehaviour = std::variant<Seek,Align>;
    SteeringBehaviour behaviour_;
    float weight_;
  };

  std::vector<BehaviourAndWeight> behaviours_;

  float maxAcceleration_;
  float maxAngularAcceleration_;

  SteeringOutput getSteering() const
  {
    SteeringOutput result{};

    for (auto &b : behaviours_)
      result +=
        b.weight_ *
        std::visit([](auto &b) { return b.getSteering(); }, b.behaviour_);

    if (result.linear_.length() > maxAcceleration_)
    {
      result.linear_.normalise();
      result.linear_ *= maxAcceleration_;
    }

    float angularAcceleration = std::abs(result.angular_);
    if (angularAcceleration > maxAngularAcceleration_)
    {
      result.angular_ /= angularAcceleration;
      result.angular_ *= maxAngularAcceleration_;
    }

    return result;
  }
};

/**
* I tried to do a thread with several parameters (didn't work) (I am used to use pthread on Linux...)
* 
* 
struct camera_params {
    cv::Mat frame, mini_frame; 
    cv::UMat gray, prevgray; //uflow;
    cv::VideoCapture vid; 
};

void camera( struct camera_params params)
{
    params.vid >> params.frame;
    cv::resize(params.frame, params.mini_frame, cv::Size(), 0.5, 0.5); // 1/3 x 2 feels more "real" in term of action but 0.5 x 2 is easier for control
    cvtColor(params.mini_frame, params.gray, cv::COLOR_BGR2GRAY);

}
*/


int main(int argc, char *argv[])
{
  int w{1024}, h{768};

  raylib::Window window(w, h, "Ai Assignment 2 B00397557");

  raylib::AudioDevice audiodevice;

  raylib::Sound fx("../resources/weird.wav"); //aoutch you got caught !
  raylib::Sound ohoh("../resources/spring.wav"); //oupsy you ran in a dark circle
  raylib::Sound money("../resources/coin.wav"); //More score for you ! YES !
  raylib::Sound sound("../resources/sound.wav"); //don't like it but eeeh that's only one left

  raylib::Music music("../resources/mini1111.xm"); //Initialize the music variable
  music.Play(); //starts the music
  
  //Creating a score, lives of the player and the highscore 
  int lives = 3;
  int score = 0;
  int highscore = 0;

  //file to read the highscore of the game before
  std::ifstream inhighscore("../resources/highscore_file.txt", std::ifstream::binary);

  // get size of file 
  inhighscore.seekg(0, inhighscore.end);
  int length = inhighscore.tellg();
  inhighscore.seekg(0, inhighscore.beg);

  //if the file isn't empty get the highscore and close the file
  if (length > 0)
  {
      inhighscore >> highscore;
  }
  inhighscore.close();

  SetTargetFPS(60);

  Ship enemy{w/2.0f + 50, h/2.0f, 0, RED};
  Ship player{w/2.0f + 250, h/2.0f + 300, 270*DEG2RAD, BLUE};

  //creation of the bank of grounds
  std::vector<Ground> grounds;
  
  float target_radius{5};
  float slow_radius{60};
  const float max_accel{200};
  const float max_ang_accel{10};
  const float max_speed{220};
  const float drag_factor{0.5};
  const float collision_distance{10};

  Seek seek{enemy.k_, player.k_, 1000};
  Align align{enemy.k_, player.k_, max_ang_accel, 1, 0.01, 0.1};
  Onward onward{player.k_, max_accel, max_ang_accel, 0};
  BlendedSteering blend{{{seek,0.5},{align,0.5}}, max_accel, max_ang_accel};

  cv::VideoCapture vid(0);
  if (!vid.isOpened()) {
    std::cerr << "error: Camera 0 could not be opened for capture.\n";
    return -1;
  }

  cv::Mat  frame, mini_frame;
  cv::UMat gray, prevgray, uflow; // Change to cv::Mat if you see OpenCL errors

  /*
  //creation of a list of parameters for the thread
  struct camera_params params;
  params.frame = frame;
  params.mini_frame = mini_frame;
  params.gray = gray;
  params.prevgray = prevgray;
  //params.uflow = uflow;
  params.vid = vid;
  */
  //start the chrono of the game
  auto start = std::chrono::steady_clock::now();

      while (!window.ShouldClose()) // Detect window close button or ESC key
      {
          if (lives > 0)
          {
              
              /*std::thread t2 = std::thread(camera, (void*)& params);

              t2.join();
              frame = params.frame;
              mini_frame = params.mini_frame;
              gray = params.gray;
              prevgray = params.prevgray;
              vid = params.vid;*/

              vid >> frame;
              cv::resize(frame, mini_frame, cv::Size(), 0.5, 0.5); // 1/3 x 2 feels more "real" in term of action but 0.5 x 2 is easier for control and for the neck !
              cvtColor(mini_frame, gray, cv::COLOR_BGR2GRAY);
              if (!prevgray.empty())
              {
                  calcOpticalFlowFarneback(prevgray, gray, uflow, 0.5, 3, 20, 3, 5, 1.2, 0);
                  cv::Scalar v = cv::mean(uflow);
                  onward.setHeadForce(v[0]);
              }


          //std::cout << GetFPS() << std::endl; //to get the FPS (Frame Rates)

          music.Update(); //launch the music in the game

          BeginDrawing();

          ClearBackground(RAYWHITE);

          auto sep = player.k_.position_ - enemy.k_.position_;
          if (sep.length() <= collision_distance)
              fx.Play();

          player.draw(w, h);
          enemy.draw(w, h);

          auto it = grounds.begin();

          //Pops a new ground everytime the frame time gets to 58ms
          if ((int)(GetFrameTime() * 1000) > 58)
          {
              //change the radius by adding the size and the minus in this constructor
              Ground ground(rand() % (w + 1), rand() % (h + 1)); 
              
              grounds.insert(it, ground);
          }

          for (int i = 0; i < grounds.size(); i++)
          {
              Ground g = grounds[i].collision(player.k_, money, ohoh, w, h, score, lives, collision_distance);
              grounds[i] = g;
              grounds[i].collision_hunter(enemy.k_,sound, w, h, score, collision_distance);
              grounds[i].draw(w, h);
          }
          player.Caught(enemy, w, h, lives, collision_distance);

          //mute that part if you only want the points of the grounds MORE DIFFICULT !
          score += GetFrameTime() * 100;


          //All of this Draw functions under are for the lines and text writing of the game 
          DrawText(TextFormat("Score: %08i", score), 30, 20, 20, RED);
          DrawLine(0, 45, 195, 45, RED);

          DrawText(TextFormat("HiScore: %08i", highscore), 30, 50, 20, GREEN);
          DrawLine(0, 70, 210, 70, GREEN);

          DrawText(TextFormat("Lives: %02i", lives), 30, 80, 20, BLUE);
          DrawLine(0, 100, 120, 100, BLUE);

          //pause the chrono and making the difference to use it
          auto end = std::chrono::steady_clock::now();
          std::chrono::duration<double> elapsed_seconds = end - start;

          DrawText(TextFormat("Elapsed Time: %04.03f s", elapsed_seconds), 30, 110, 20, BLACK);
          DrawLine(0, 130, 250, 130, BLACK);
          EndDrawing();

          auto se = blend.getSteering();
          auto sp = onward.getSteering();
          player.update(sp, max_speed - 60, drag_factor, GetFrameTime());
          enemy.update(se, max_speed, drag_factor, GetFrameTime());

          std::swap(prevgray, gray);
      }
      else {
              if (score > highscore) { highscore = score; }
              //file to save the highscore as a bonus for next game
              std::ofstream outhighscore("../resources/highscore_file.txt", std::ofstream::binary);
              outhighscore << highscore;
              outhighscore.close();

              //restart the music
              music.Stop();
              music.Play();

              //reset lives, score and grounds
              lives = 3;
              score = 0;
              grounds.erase(grounds.begin(), grounds.end());
              //restart the chrono of the game
              start = std::chrono::steady_clock::now();
          }
  }

  return 0;
}
