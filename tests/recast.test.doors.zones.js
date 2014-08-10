/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports, require, THREE, TWEEN, Stats */
'use strict';

var renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize(document.body.clientWidth * 0.6, document.body.clientHeight * 0.8);
renderer.domElement.style.position = 'absolute';
renderer.domElement.style.bottom = 0;
renderer.domElement.style.right = 0;
document.body.appendChild(renderer.domElement);
renderer.setClearColorHex(0xFFFFFF, 1.0);
renderer.clear();

var width = renderer.domElement.width;
var height = renderer.domElement.height;
var camera = new THREE.PerspectiveCamera( 45, width / height, 1, 10000);
camera.position.y = 50;
camera.position.z = 50;

var controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.addEventListener('change', function(){
    render();
});

var scene = new THREE.Scene();

var agent = new THREE.Object3D();
var agentBody = new THREE.Mesh(
    new THREE.CylinderGeometry(0.2, 0.5, 2),
    new THREE.MeshBasicMaterial({
      color: '#FF0000'
    })
);
agentBody.position.y = 1;
agent.add(agentBody);

agent.arrowHelper = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 1),
    new THREE.Vector3(0, 0, 0),
    1.5,
    0xffff00);
agent.add(agent.arrowHelper);

scene.add(agent);

var light = new THREE.SpotLight();
light.position.set( 170, 330, -160 );
scene.add(light);

////////////////////////////////

var terrain, navigationMesh, doors = {};
var debugDraw = {};
var recast = require('../lib/recast');

recast.setGLContext(renderer.context);

function render () {
    renderer.render(scene, camera);

    if (debugDraw.NavMesh)              { recast.drawObject('NavMesh');             }                        
    if (debugDraw.NavMeshPortals)       { recast.drawObject('NavMeshPortals');      }          
    if (debugDraw.RegionConnections)    { recast.drawObject('RegionConnections');   }    
    if (debugDraw.RawContours)          { recast.drawObject('RawContours');         }                
    if (debugDraw.Contours)             { recast.drawObject('Contours');            }                      
    if (debugDraw.HeightfieldSolid)     { recast.drawObject('HeightfieldSolid');    }      
    if (debugDraw.HeightfieldWalkable)  { recast.drawObject('HeightfieldWalkable'); }
}

function addMeshFromVertices (vertices, parent, plain) {
    var materials = [ new THREE.MeshBasicMaterial({
        color: 0xFF0000,
        shading: THREE.FlatShading,
        side: THREE.DoubleSide,
        wireframe: ! plain,
        transparent: true,
        opacity: 0.3,
        overdraw: true
    }) ];

    for (var i = 0; i < vertices.length; i++) {
        if (!vertices[i+2]) { break; }

        var geometry = new THREE.ConvexGeometry([
            new THREE.Vector3(   vertices[i].x,   vertices[i].y,   vertices[i].z ), 
            new THREE.Vector3( vertices[i+1].x, vertices[i+1].y, vertices[i+1].z ),
            new THREE.Vector3( vertices[i+2].x, vertices[i+2].y, vertices[i+2].z )
        ]);

        var child = THREE.SceneUtils.createMultiMaterialObject(geometry, materials);
        parent.add(child);

        i += 2;
    }
}

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
exports['handle doors'] = function(test) {
    test.expect(11);

    recast.settings({
        cellSize: 0.1,
        cellHeight: 0.05,
        agentHeight: 1.8,
        agentRadius: 0.4,
        agentMaxClimb: 2.0,
        agentMaxSlope: 30.0
    });


    var stats = new Stats();
    stats.setMode(1); // 0: fps, 1: ms
    stats.domElement.style.position = 'absolute';
    stats.domElement.style.right = '0px';
    stats.domElement.style.bottom = '0px';
    document.body.appendChild( stats.domElement );
    stats.setMode(0);
    stats.begin();

   
    var loader = new THREE.OBJLoader();
    loader.load('doors.obj', function(object){
        terrain = object;

        scene.add(object);

        /**
         * Load an .OBJ file
         */
        recast.OBJLoader('doors.obj', function(){

            /**
             * Get navmesh geometry and draw it
             */
            if (location.search.match(/navigationmesh=1/)) {
                recast.getNavMeshVertices(recast.cb(function (vertices) {
                    navigationMesh = new THREE.Object3D();
                    addMeshFromVertices(vertices, navigationMesh);
                }));
            }

            /**
             * Load zones
             */
            recast.setZones({
                door1: { name: 'door1', refs: [ 69  ], flags: [ 16 ] },
                door2: { name: 'door2', refs: [ 103 ], flags: [ 16 ] },
                door3: { name: 'door3', refs: [ 84  ], flags: [ 16 ] }
            });

            // for (var door in recast.zones) {
            //     door = recast.zones[door];
            //     var doorGeometry = new THREE.BoxGeometry(doorSize.x, 6, doorSize.z);
            //     for (var i = 0; i < doorGeometry.vertices.length; i++) {
            //         // doorGeometry.vertices[i].x += 2.5;
            //         doorGeometry.vertices[i].y += 3;
            //     }
            //     doors[child.name].doorObject = new THREE.Mesh(
            //         doorGeometry,
            //         new THREE.MeshBasicMaterial({
            //             color: 0x050505,
            //             side: THREE.DoubleSide,
            //             overdraw: true
            //         })
            //     );
            //     doors[child.name].doorObject.position = geometry.boundingSphere.center;
            //     scene.add(doors[child.name].doorObject);
            // }

            recast.vent.on('update', function (agents) {
                agent.position.set(
                    agents[0].position.x, 
                    agents[0].position.y, 
                    agents[0].position.z
                );
            });

            var hop = 0,
                hops = [
                {
                    x: 87.5,
                    y: 0,
                    z: 10.8 // -15
                },
                {
                    x: 1.6,
                    y: 0,
                    z: -12.9 // -15
                },
                {
                    x: 43.9,
                    y: 0,
                    z: 9.6 // -15
                },
                {
                    x: 80.1,
                    y: 0,
                    z: -11 // -15
                }
            ];

            recast.addAgent({
                position: hops[3],
                radius: 0.8,
                height: 0.5,
                maxAcceleration: 1.0,
                maxSpeed: 2.0,
                updateFlags: 0, // && recast.CROWD_OBSTACLE_AVOIDANCE, // & recast.CROWD_ANTICIPATE_TURNS & recast.CROWD_OPTIMIZE_TOPO & recast.CROWD_SEPARATION,
                separationWeight: 20.0
            });

            var last = new Date().getTime();
            var animate = function animate (time) {
                window.requestAnimationFrame(animate);

                TWEEN.update();
                recast.crowdUpdate(0.1);
                recast.crowdGetActiveAgents();

                last = time;
                render();

                if (stats) stats.update();
            };

            var move = function (index) {
                var hop = index !== undefined ? index : hops[~~(Math.random() * hops.length)];
                recast.crowdRequestMoveTarget(0, hop.x, hop.y, hop.z);
            };

            var toggleDoor = function (doorName) {
                recast.zones[doorName].toggleFlags(recast.FLAG_DISABLED);
                // var door = recast.zones[doorName];
                // // door.doorObject.rotation.y = door.closed ? 0 : 2;
                // new TWEEN.Tween(door.doorObject.rotation)
                //     .to({
                //         y: door.closed ? 0 : 2
                //     }, 500)
                //     .start();

                // door.closed = ! door.closed;                    
            };
            window.toggleDoor = toggleDoor;

            move();
            setInterval(move, 5 * 1000);

            animate(new Date().getTime());
        });
    });
};
