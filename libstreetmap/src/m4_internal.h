/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   m4_internal.h
 * Author: wangh365
 *
 * Created on April 8, 2021, 3:48 PM
 */

#pragma once //protects against multiple inclusions of this header file

#include "m4.h"
#include "m3_internal.h"
#include "unordered_set"
#include "unordered_map"
#include <chrono>
#include <cmath>

bool checkLegalPath(std::vector<CourierSubPath> path, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots);

//multi dijstra single source algorithm 
void multiDestDijstra(int intersect_id_start,const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, const float turn_penalty, int numInterestingIntersections, std::unordered_map <IntersectionIdx, bool> isPickup);

//function that selects the optimal starting depot
IntersectionIdx findStartingLocation(const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots);

void findPermutationsHelper(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
void findPermutations(std::vector<IntersectionIdx> current, std::vector<IntersectionIdx> all);
std::vector<int>  checkIfPickedUp(int id, std::unordered_map <IntersectionIdx, std::unordered_set <IntersectionIdx>> dropOffToPickUp, std:: unordered_set <IntersectionIdx> alreadyPickedUp);
std::pair<std::vector<CourierSubPath>, double> trySwap(std::vector<CourierSubPath> path,int i ,double time,  double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots);

std::pair<std::vector<CourierSubPath>, double>  perturbation(std::vector<CourierSubPath> path, int i, double time, double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots);

std::vector<CourierSubPath> findCourierPath(IntersectionIdx begin, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots, const float turn_penalty,int og);

double calucauteTotalTravelTime(std::vector<CourierSubPath> &path);
std::pair<std::vector<CourierSubPath>, double> trySwap2(std::vector<CourierSubPath> path,int i ,double time,  double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots);
void perturbLongPaths(std::vector<CourierSubPath> &path, int index, int lookAheadNum, double time,  
        double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots);
void perturbLongPathsHelper(std::vector<CourierSubPath> &path, int index, int lookAheadNum, double time,  
        double turn_penalty, const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots,
        std::vector<int> current, std::unordered_set<int> used);
//function that checks the legality of a sub path
bool checkLegalSubPath(std::vector<CourierSubPath> path, const std::vector<DeliveryInf>& deliveries);


