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

var recast   = require('../lib/recast');
var settings = require('./settings');

exports['manage the crowd'] = function(test) {

    test.expect(4);

    settings(recast);

    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function () {

        recast.buildTiled();

        recast.initCrowd(1000, 1.0);

        /**
         * Find a random navigable point on this mesh
         */
        recast.getRandomPoint(recast.cb(function(pt1x, pt1y, pt1z){

            test.ok(typeof pt1x === 'number' && typeof pt1y === 'number' && typeof pt1z === 'number', 'find a random point');

            /**
             * Add an agent, retain its ID
             */
            var id = recast.addAgent({
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
            });

            test.ok(typeof id === 'number', 'agent ID is a number');


            recast.vent.on('update', function (agents) {
                test.ok(agents, 'crowd should contain agents');
                test.strictEqual(agents.length, 1, 'crowd should contain exactly 1 agent');
                test.done();
            });

            recast.crowdUpdate(1.0);
            recast.crowdGetActiveAgents();

            // TODO
            // removeCrowdAgent
            // crowdUpdate
            // crowdGetActiveAgents
        }));
    });
};


