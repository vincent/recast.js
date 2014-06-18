/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports: true, require: true */
'use strict';

var recast = require('../lib/recast.withworker');

var ENVIRONMENT_IS_NODE = typeof process === 'object' && typeof require === 'function';
var ENVIRONMENT_IS_WEB = typeof window === 'object';
var ENVIRONMENT_IS_WORKER = typeof importScripts === 'function';
var ENVIRONMENT_IS_SHELL = !ENVIRONMENT_IS_WEB && !ENVIRONMENT_IS_NODE && !ENVIRONMENT_IS_WORKER;

recast = new recast('../lib/recast');

// Check our library is here
exports['recast is present'] = function(test) {
    test.ok(recast, 'recast should be an object');
    test.done();
};

// Check our methods are here
exports['our methods are present'] = function(test) {
    test.ok(recast.set_cellSize, 'set_cellSize');
    test.ok(recast.set_cellHeight, 'set_cellHeight');
    test.ok(recast.set_agentHeight, 'set_agentHeight');
    test.ok(recast.set_agentRadius, 'set_agentRadius');
    test.ok(recast.set_agentMaxClimb, 'set_agentMaxClimb');
    test.ok(recast.set_agentMaxSlope, 'set_agentMaxSlope');

    test.ok(recast.build, 'build');
    test.ok(recast.initCrowd, 'initCrowd');
    test.ok(recast.initWithFileContent, 'initWithFileContent');
    test.ok(recast.findNearestPoint, 'findNearestPoint');
    test.ok(recast.findPath, 'findPath');
    test.ok(recast.getRandomPoint, 'getRandomPoint');

    test.ok(recast.addCrowdAgent, 'addCrowdAgent');
    test.ok(recast.updateCrowdAgentParameters, 'updateCrowdAgentParameters');
    test.ok(recast.requestMoveVelocity, 'requestMoveVelocity');
    test.ok(recast.removeCrowdAgent, 'removeCrowdAgent');
    test.ok(recast.crowdRequestMoveTarget, 'crowdRequestMoveTarget');
    test.ok(recast.crowdUpdate, 'crowdUpdate');
    test.ok(recast.crowdGetActiveAgents, 'crowdGetActiveAgents');
    test.done();
};

// Check file loading
exports['load an .obj file'] = function(test) {
    test.expect(9);

    recast.set_cellSize(1.0);
    recast.set_cellHeight(2.0);
    recast.set_agentHeight(2.0);
    recast.set_agentRadius(0.2);
    recast.set_agentMaxClimb(4.0);
    recast.set_agentMaxSlope(30.0);

    /*
    recast.settings({
        cellSize: 2.0,
        cellHeight: 1.5,
        agentHeight: 2.0,
        agentRadius: 0.2,
        agentMaxClimb: 4.0,
        agentMaxSlope: 30.0
    });
    */
   
    /**
     * Load an .OBJ file
     */
    recast.OBJLoader(ENVIRONMENT_IS_WEB ? '/tests/nav_test.obj' : '../tests/nav_test.obj', recast.cb(function(){

        /**
         * Find a random navigable point on this mesh
         */
        recast.getRandomPoint(recast.cb(function(pt1x, pt1y, pt1z){
            test.ok(typeof pt1x === 'number', 'point coord is a number');
            test.ok(typeof pt1y === 'number', 'point coord is a number');
            test.ok(typeof pt1z === 'number', 'point coord is a number');

            /**
             * Find the nearest navigable point from 0,0,0 with a maximum extend of 10,10,10
             */
            recast.findNearestPoint(0, 0, 0, 10, 10, 10, recast.cb(function(pt2x, pt2y, pt2z){
                test.ok(typeof pt2x === 'number', 'point coord is a number');
                test.ok(typeof pt2y === 'number', 'point coord is a number');
                test.ok(typeof pt2z === 'number', 'point coord is a number');

                var extend = 10;

                /**
                 * Find the nearest navigable polygon from 0,0,0 with a maximum extend of 10
                 */
                recast.findNearestPoly(0, 0, 0, extend, extend, extend, recast.cb(function(count, vertice1, vertice2 /* ... */){
                    test.ok(count > 0, 'origin poly has some vertices');

                    var vertices = Array.prototype.slice.call(arguments, 1);
                    test.ok(vertices && typeof vertices.length !== 'undefined', 'origin poly has ' + vertices.length + ' vertices');

                    /**
                     * Find the shortest possible path from pt1 to pt2
                     */
                    recast.findPath(pt1x, pt1y, pt1z, pt2x, pt2y, pt2z, 1000, recast.cb(function(path){
                        test.ok(path && typeof path.length !== 'undefined', 'found path has ' + path.length + ' segments');

                        test.done();
                    }));
                }));
            }));

        }));
    }));
};


// Check file loading
exports['manage the crowd'] = function(test) {
    test.expect(6);

    recast.set_cellSize(1.0);
    recast.set_cellHeight(2.0);
    recast.set_agentHeight(2.0);
    recast.set_agentRadius(0.2);
    recast.set_agentMaxClimb(4.0);
    recast.set_agentMaxSlope(30.0);

    /*
    recast.settings({
        cellSize: 2.0,
        cellHeight: 1.5,
        agentHeight: 2.0,
        agentRadius: 0.2,
        agentMaxClimb: 4.0,
        agentMaxSlope: 30.0
    });
    */
   
    /**
     * Load an .OBJ file
     */
    recast.OBJLoader(ENVIRONMENT_IS_WEB ? '/tests/nav_test.obj' : '../tests/nav_test.obj', recast.cb(function () {

        recast.vent.on('update', function (agents) {
            test.ok(agents && typeof agents.length !== 'undefined', 'crowd has ' + agents.length + ' agent');
            test.strictEqual(agents.length, 1);
        });

        /**
         * Find a random navigable point on this mesh
         */
        recast.getRandomPoint(recast.cb(function(pt1x, pt1y, pt1z){
            test.ok(typeof pt1x === 'number', 'point coord is a number');
            test.ok(typeof pt1y === 'number', 'point coord is a number');
            test.ok(typeof pt1z === 'number', 'point coord is a number');

            /**
             * Add an agent, retain its ID
             */
            recast.addAgent({
                position: {
                    x: pt1x,
                    y: pt1y,
                    z: pt1z
                },
                radius: 0.5,
                height: 0.8,
                maxAcceleration: 1.0,
                maxSpeed: 2.0,
                updateFlags: 0,
                separationWeight: 10.0
            }, recast.cb(function(agent_id){

                test.ok(typeof agent_id === 'number', 'agent ID is a number');

                recast.crowdUpdate(1.0);

                setTimeout(function(){
                    recast.crowdUpdate(1.0, recast.cb(function(agents){
                        test.strictEqual(agents.length, 1);

                        setTimeout(function(){
                            recast.crowdGetActiveAgents(recast.cb(function(agents){
                                test.strictEqual(agents.length, 1);
                                test.done();
                            }));
                        }, 2000);

                    }));
                }, 1000);

                // removeCrowdAgent

            }));

        }));
    }));
};