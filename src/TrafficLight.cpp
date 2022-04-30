#include "TrafficLight.h"
#include <future>
#include <iostream>
#include <random>
#include <chrono>

/* Implementation of class "MessageQueue" */


template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : DONE
    std::unique_lock<std::mutex> lock(_mutex);
    _condition.wait(lock, [this] { return !_queue.empty(); });
    auto returnvalue = std::move(_queue.back());
    _queue.clear();
    return returnvalue;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    //FP.4a - DONE
    std::lock_guard<std::mutex> lguard(_mutex);
    _queue.push_back(std::move(msg));
    _condition.notify_one();
}


/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight() {
  _currentPhase = TrafficLightPhase::red;
  _messages = std::make_shared<MessageQueue<TrafficLightPhase>>(); }

TrafficLight::~TrafficLight() { }

void TrafficLight::waitForGreen()
{
    // FP.5b : DONE
    while(true) {
        auto phase = _messages->receive();
        if (phase == green) {
            return;
        }
    }
}
TrafficLightPhase TrafficLight::getCurrentPhase() { return _currentPhase; }

void TrafficLight::simulate() {
  // FP.2b : DONE
  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));

}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases() {
  // FP.2a - DONE 

  auto start = std::chrono::system_clock::now();

  std::uniform_real_distribution<double> uni_dist(4.0, 6.0);
  std::default_random_engine random_eng(std::random_device{}());
  double minutes_elapsed = uni_dist(random_eng);

  while (true) {

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    
    auto loop_time = std::chrono::system_clock::now();
    std::chrono::duration<double> seconds_elapsed = loop_time - start;

    if (seconds_elapsed.count() >= minutes_elapsed) {

      if (_currentPhase == green) {
        _currentPhase = red;
      } else {
        _currentPhase = green;
      }

      // adding new phase to message queue
      auto message = _currentPhase;
      auto f_send =
          std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send,
                     _messages, std::move(message));
      f_send.wait();

      minutes_elapsed = uni_dist(random_eng);
      start = std::chrono::system_clock::now();
    }
  }
}
