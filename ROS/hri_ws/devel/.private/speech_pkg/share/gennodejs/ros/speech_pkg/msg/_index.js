
"use strict";

let Gesture = require('./Gesture.js');
let Speech = require('./Speech.js');
let IntentIRI = require('./IntentIRI.js');
let Command = require('./Command.js');
let SpeechData = require('./SpeechData.js');
let SystemHealth = require('./SystemHealth.js');
let IntentActionResult = require('./IntentActionResult.js');
let IntentAction = require('./IntentAction.js');
let IntentActionGoal = require('./IntentActionGoal.js');
let IntentResult = require('./IntentResult.js');
let IntentActionFeedback = require('./IntentActionFeedback.js');
let IntentGoal = require('./IntentGoal.js');
let IntentFeedback = require('./IntentFeedback.js');

module.exports = {
  Gesture: Gesture,
  Speech: Speech,
  IntentIRI: IntentIRI,
  Command: Command,
  SpeechData: SpeechData,
  SystemHealth: SystemHealth,
  IntentActionResult: IntentActionResult,
  IntentAction: IntentAction,
  IntentActionGoal: IntentActionGoal,
  IntentResult: IntentResult,
  IntentActionFeedback: IntentActionFeedback,
  IntentGoal: IntentGoal,
  IntentFeedback: IntentFeedback,
};
