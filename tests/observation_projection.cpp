#include <iostream>
#include "observations.hpp"
using namespace egcs2017;
int main() {
    Building b; b.floors=6;b.capacity=13;
    CarView car;car.id=0;car.load=4;car.car_calls={{0,1},{3,3}};
    ObservedCall call;call.view.key={4,DOWN};call.view.registered=7;call.view.assigned=0;call.view.waiting=2;
    std::vector<CarView> cars{car};std::vector<ObservedCall> calls{call};
    auto original=observation_json(b,10,cars,calls);
    cars[0].car_calls[0]=999;cars[0].car_calls[3]=777;
    calls[0].view.waiting=10000;calls[0].view.waited=1234;calls[0].view.forbidden_car=7;
    if(original!=observation_json(b,10,cars,calls)) return 2;
    cars[0].load=5;
    if(original==observation_json(b,10,cars,calls)) return 3;
    std::cout<<original<<'\n';
}
