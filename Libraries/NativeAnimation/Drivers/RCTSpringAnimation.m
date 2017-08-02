/**
 * Copyright (c) 2015-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree. An additional grant
 * of patent rights can be found in the PATENTS file in the same directory.
 */

#import "RCTSpringAnimation.h"

#import <UIKit/UIKit.h>

#import <React/RCTConvert.h>
#import <React/RCTDefines.h>

#import "RCTValueAnimatedNode.h"

@interface RCTSpringAnimation ()

@property (nonatomic, strong) NSNumber *animationId;
@property (nonatomic, strong) RCTValueAnimatedNode *valueNode;
@property (nonatomic, assign) BOOL animationHasBegun;
@property (nonatomic, assign) BOOL animationHasFinished;

@end

typedef NS_ENUM(NSUInteger, RCTSpringAnimationModel) {
  RCTSpringAnimationModelRK4, // Runge-Kutta 4th order integration model
  RCTSpringAnimationModelDHO // damped harmonic spring model
};

@implementation RCTSpringAnimation
{
  CGFloat _toValue;
  CGFloat _fromValue;
  BOOL _overshootClamping;
  CGFloat _restDisplacementThreshold;
  CGFloat _restSpeedThreshold;
  CGFloat _tension;
  CGFloat _friction;
  CGFloat _stiffness;
  CGFloat _damping;
  CGFloat _mass;
  RCTSpringAnimationModel _animationModel;
  CGFloat _initialVelocity;
  NSTimeInterval _animationStartTime;
  NSTimeInterval _animationCurrentTime;
  RCTResponseSenderBlock _callback;

  CGFloat _lastPosition;
  CGFloat _lastVelocity;

  NSInteger _iterations;
  NSInteger _currentLoop;
  
  CGFloat _t; // Current time (startTime + dt) used for DHO model
}

- (instancetype)initWithId:(NSNumber *)animationId
                    config:(NSDictionary *)config
                   forNode:(RCTValueAnimatedNode *)valueNode
                  callBack:(nullable RCTResponseSenderBlock)callback
{
  if ((self = [super init])) {
    NSNumber *iterations = [RCTConvert NSNumber:config[@"iterations"]] ?: @1;

    _animationId = animationId;
    _toValue = [RCTConvert CGFloat:config[@"toValue"]];
    _fromValue = valueNode.value;
    _lastPosition = 0;
    _valueNode = valueNode;
    _overshootClamping = [RCTConvert BOOL:config[@"overshootClamping"]];
    _restDisplacementThreshold = [RCTConvert CGFloat:config[@"restDisplacementThreshold"]];
    _restSpeedThreshold = [RCTConvert CGFloat:config[@"restSpeedThreshold"]];
    // RK4 model
    if (config[@"tension"] != nil) {
      _tension = [RCTConvert CGFloat:config[@"tension"]];
      _friction = [RCTConvert CGFloat:config[@"friction"]];
      _animationModel = RCTSpringAnimationModelRK4;
    } else if (config[@"stiffness"] != nil) { // DHO model
      _stiffness = [RCTConvert CGFloat:config[@"stiffness"]];
      _damping = [RCTConvert CGFloat:config[@"damping"]];
      _mass = [RCTConvert CGFloat:config[@"mass"]];
      _animationModel = RCTSpringAnimationModelDHO;
    }
    _initialVelocity = [RCTConvert CGFloat:config[@"initialVelocity"]];
    
    _callback = [callback copy];

    _lastPosition = _fromValue;
    _lastVelocity = _initialVelocity;

    _animationHasFinished = iterations.integerValue == 0;
    _iterations = iterations.integerValue;
    _currentLoop = 1;
  }
  return self;
}

RCT_NOT_IMPLEMENTED(- (instancetype)init)

- (void)startAnimation
{
  _animationStartTime = _animationCurrentTime = -1;
  _animationHasBegun = YES;
}

- (void)stopAnimation
{
  _valueNode = nil;
  if (_callback) {
    _callback(@[@{
      @"finished": @(_animationHasFinished)
    }]);
  }
}

- (void)stepAnimationWithTime:(NSTimeInterval)currentTime
{
  if (!_animationHasBegun || _animationHasFinished) {
    // Animation has not begun or animation has already finished.
    return;
  }
  
  if (_animationModel == RCTSpringAnimationModelRK4) {
    [self stepAnimationRK4WithTime:currentTime];
  } else {
    [self stepAnimationDHOWithTime:currentTime];
  }
}

- (void)stepAnimationDHOWithTime:(NSTimeInterval)currentTime
{
  if (!_animationHasBegun || _animationHasFinished) {
    // Animation has not begun or animation has already finished.
    return;
  }
  
  // calculate delta time
  CFTimeInterval deltaTime;
  if(_animationStartTime == -1) {
    _t = 0.0;
    _animationStartTime = currentTime;
    deltaTime = 0.0;
  } else {
    deltaTime = currentTime - _animationCurrentTime;
    _t = _t + deltaTime;
  }
  
  // store the timestamp
  _animationCurrentTime = currentTime;
  
  CGFloat c = _damping;
  CGFloat m = _mass;
  CGFloat k = _stiffness;
  CGFloat v0 = _initialVelocity;
  
  NSParameterAssert(m > 0);
  NSParameterAssert(k > 0);
  NSParameterAssert(c > 0);
  
  CGFloat zeta = c / (2 * sqrtf(k * m));
  CGFloat omega0 = sqrtf(k / m);
  CGFloat omega1 = omega0 * sqrtf(1.0 - (zeta * zeta));
  
  CGFloat x0 = 1;
  
  CGFloat (^oscillation)(CGFloat);
  if (zeta < 1) {
    // Underdamped
    oscillation = ^(CGFloat t) {
      CGFloat envelope = expf(-zeta * omega0 * t);
      return envelope * (((v0 + zeta * omega0 * x0) / omega1) * sinf(omega1 * t) + (x0 * cosf(omega1 * t)));
    };
  } else {
    // Critically damped
    oscillation = ^(CGFloat t) {
      CGFloat envelope = expf(-omega0 * t);
      return envelope * (x0 + (v0 + (omega0 * x0)) * t);
    };
  }
    
  CGFloat delta = _toValue - _fromValue;
  CGFloat fraction = 1 - oscillation(_t);
  CGFloat position = (_fromValue + fraction * delta);
  
  [self onUpdate:position];
  
  CGFloat velocity = (position - _lastPosition) / deltaTime;
  
  // Conditions for stopping the spring animation
  BOOL isOvershooting = NO;
  if (_overshootClamping && _stiffness != 0) {
    if (_fromValue < _toValue) {
      isOvershooting = position > _toValue;
    } else {
      isOvershooting = position < _toValue;
    }
  }
  BOOL isVelocity = ABS(velocity) <= _restSpeedThreshold;
  BOOL isDisplacement = YES;
  if (_stiffness != 0) {
    isDisplacement = ABS(_toValue - position) <= _restDisplacementThreshold;
  }
  
  if (isOvershooting || (isVelocity && isDisplacement)) {
    if (_stiffness != 0) {
      // Ensure that we end up with a round value
      if (_animationHasFinished) {
        return;
      }
      [self onUpdate:_toValue];
    }
    
    if (_iterations == -1 || _currentLoop < _iterations) {
      _lastPosition = _fromValue;
      _lastVelocity = _initialVelocity;
      // Set _animationStartTime to -1 to reset instance variables on the next animation step.
      _animationStartTime = -1;
      _currentLoop++;
      [self onUpdate:_fromValue];
    } else {
      _animationHasFinished = YES;
    }
  }
  
  _lastPosition = position;
}

- (void)stepAnimationRK4WithTime:(NSTimeInterval)currentTime
{
  if (_animationStartTime == -1) {
    _animationStartTime = _animationCurrentTime = currentTime;
  }

  // We are using a fixed time step and a maximum number of iterations.
  // The following post provides a lot of thoughts into how to build this
  // loop: http://gafferongames.com/game-physics/fix-your-timestep/
  CGFloat TIMESTEP_MSEC = 1;
  // Velocity is based on seconds instead of milliseconds
  CGFloat step = TIMESTEP_MSEC / 1000;

  NSInteger numSteps = floorf((currentTime - _animationCurrentTime) / step);
  _animationCurrentTime = currentTime;
  if (numSteps == 0) {
    return;
  }

  CGFloat position = _lastPosition;
  CGFloat velocity = _lastVelocity;

  CGFloat tempPosition = _lastPosition;
  CGFloat tempVelocity = _lastVelocity;

  for (NSInteger i = 0; i < numSteps; ++i) {
    // This is using RK4. A good blog post to understand how it works:
    // http://gafferongames.com/game-physics/integration-basics/
    CGFloat aVelocity = velocity;
    CGFloat aAcceleration = _tension * (_toValue - tempPosition) - _friction * tempVelocity;
    tempPosition = position + aVelocity * step / 2;
    tempVelocity = velocity + aAcceleration * step / 2;

    CGFloat bVelocity = tempVelocity;
    CGFloat bAcceleration = _tension * (_toValue - tempPosition) - _friction * tempVelocity;
    tempPosition = position + bVelocity * step / 2;
    tempVelocity = velocity + bAcceleration * step / 2;

    CGFloat cVelocity = tempVelocity;
    CGFloat cAcceleration = _tension * (_toValue - tempPosition) - _friction * tempVelocity;
    tempPosition = position + cVelocity * step / 2;
    tempVelocity = velocity + cAcceleration * step / 2;

    CGFloat dVelocity = tempVelocity;
    CGFloat dAcceleration = _tension * (_toValue - tempPosition) - _friction * tempVelocity;
    tempPosition = position + cVelocity * step / 2;
    tempVelocity = velocity + cAcceleration * step / 2;

    CGFloat dxdt = (aVelocity + 2 * (bVelocity + cVelocity) + dVelocity) / 6;
    CGFloat dvdt = (aAcceleration + 2 * (bAcceleration + cAcceleration) + dAcceleration) / 6;

    position += dxdt * step;
    velocity += dvdt * step;
  }

  _lastPosition = position;
  _lastVelocity = velocity;

  [self onUpdate:position];

  if (_animationHasFinished) {
    return;
  }

  // Conditions for stopping the spring animation
  BOOL isOvershooting = NO;
  if (_overshootClamping && _tension != 0) {
    if (_fromValue < _toValue) {
      isOvershooting = position > _toValue;
    } else {
      isOvershooting = position < _toValue;
    }
  }
  BOOL isVelocity = ABS(velocity) <= _restSpeedThreshold;
  BOOL isDisplacement = YES;
  if (_tension != 0) {
    isDisplacement = ABS(_toValue - position) <= _restDisplacementThreshold;
  }

  if (isOvershooting || (isVelocity && isDisplacement)) {
    if (_tension != 0) {
      // Ensure that we end up with a round value
      if (_animationHasFinished) {
        return;
      }
      [self onUpdate:_toValue];
    }

    if (_iterations == -1 || _currentLoop < _iterations) {
      _lastPosition = _fromValue;
      _lastVelocity = _initialVelocity;
      _currentLoop++;
      [self onUpdate:_fromValue];
    } else {
      _animationHasFinished = YES;
    }
  }
}

- (void)onUpdate:(CGFloat)outputValue
{
  _valueNode.value = outputValue;
  [_valueNode setNeedsUpdate];
}

@end
