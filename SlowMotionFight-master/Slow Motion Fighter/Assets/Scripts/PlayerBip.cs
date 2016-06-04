using UnityEngine;
using System.Collections;
using CartWheelCore;
using CartWheelCore.Control;

public enum cameraDirection{
	none = 0,
	left = 1,
	right = 2,
	back = 3,
	front = 4
}

public class PlayerBip : Bipedal {
		
	public TextMesh prompt;

	private int punchID = (int)BipedalActions.punch1;
	private int kickID = (int)BipedalActions.kick1;
	private int weaponPunchID = (int)BipedalActions.weaponHit1;
	
	private int moveLevel = 0;
	private float distenceToTargetEnemy = 0;
	//private EnemyBip targetEnemy = null;
	private ArrayList targetEnemys = new ArrayList();
	//private RubberBall currBall = null;
	private ArrayList balls = new ArrayList();
	private ColliderArea targetEnemyArea = ColliderArea.none;
	private cameraDirection currCameraDir = cameraDirection.none;
	private int targetEnemyChangeTimer = 0;
	private int cameraChangeTimer = 0;
	private int targetChangeTimer = 0;
	private int cameraDirSign = 1;
	private float vigilanceTimer = 0;
	private float ballMoveTimer = 0;

	override protected void startCharacter(){
		//showPrompt (false);
		startStand();
	}
	
	override protected void ResponseStartNewStateEvent(object sender, System.EventArgs e){
		for (int i=0;i<controller.getCurrentState().getTrajectoryCount();i++){
			Trajectory tra = controller.getCurrentState().getTrajectory(i);
			if(tra.hitFeedback != null){
				tra.hitFeedback.isHit = false;
				ignoreCollisionTrunkWithLowerArm(true);
				/*
				if(isIgnoreCollisionWithOthers){
					for(int j = 0; j < RFGameManager.instance.getEnemyCount(); j++){
						ignoreCollisionARBWithOtherBipedal(character.getArticulatedRigidBody(tra.hitFeedback.hitARBIndex), RFGameManager.instance.getEnemy(j), "enemyTrunk", false);
					}
				}*/
			}
		}
		
		base.ResponseStartNewStateEvent(sender, e);
	}
	
	override protected void ResponseStateEndedEvent(object sender, System.EventArgs e){
		for (int i=0;i<controller.getCurrentState().getTrajectoryCount();i++){
			Trajectory tra = controller.getCurrentState().getTrajectory(i);
			if(tra.hitFeedback != null){
				ignoreCollisionTrunkWithLowerArm(false);
				/*
				if(isIgnoreCollisionWithOthers){
					for(int j = 0; j < RFGameManager.instance.getEnemyCount(); j++){
						ignoreCollisionARBWithOtherBipedal(character.getArticulatedRigidBody(tra.hitFeedback.hitARBIndex), RFGameManager.instance.getEnemy(j));
					}
				}*/
			}
		}
		base.ResponseStateEndedEvent(sender, e);
	}
	
	override protected void ResponseHitOthersFeedback(HitFeedback feedback, ContactPointInfo cp){
		base.ResponseHitOthersFeedback(feedback, cp);
		if(!feedback.isHit){
			/*
			if (cp.collider2.tag.Contains ("enemyTrunk")) {
				//if(cp.collider2.tag == "enemyTrunk"){
				EnemyBip enemy = ((cp.collider2.GetComponent ("CWRigidBody") as CWRigidBody).getAFParent () as Character).ownerObject.GetComponent ("EnemyBip") as EnemyBip;
				
				HitFeedback hit = feedback.copy ();
				hit.feedbackARB = cp.collider2.GetComponent ("CWRigidBody") as CWRigidBody;
				if (hit.feedbackARB.name == "head") {
					hit.bodySpringKp = 30;
					hit.bodySpringKd = 5;
					hit.force /= 1.5f;
				}
				Vector3 f = Vector3.zero;
				f += character.getLocalFrontAxis () * hit.force.x;
				f += character.getLocalUpAxis () * hit.force.y;
				f += character.getLocalLeftAxis () * hit.force.z;
				f = controller.getCharacterFrame () * f;

				float levelDiff = GameAttribute.gameAttribute.loadPlayerLevel () - enemy.level;
				enemy.addLife (-1 * Mathf.Clamp (hit.power + levelDiff, 1, 100));
				hit.localPos = hit.feedbackARB.getLocalCoordinatesPoint (cp.cp);
				enemy.startHurt (hit);
				cp.collider2.attachedRigidbody.AddForceAtPosition (f, cp.cp, ForceMode.Impulse);
				//enemy.pelvis.GetComponent<Rigidbody> ().AddForce (0.5f*f, ForceMode.Impulse);
				RFGameManager.instance.addPunch (hit.feedbackARB, cp.cp);

				if (weapon == null) {
					if (cp.collider1.gameObject == lLowerArm || cp.collider1.gameObject == lLowerLeg) {
						lHandSoundSource.clip = bipedalSounds [Random.Range (2, 10)];
						lHandSoundSource.Play ();
					} else if (cp.collider1.gameObject == rLowerArm || cp.collider1.gameObject == rLowerLeg) {
						rHandSoundSource.clip = bipedalSounds [Random.Range (2, 10)];
						rHandSoundSource.Play ();
					}
				} else {
					weapon.playAttackSounds ();
					weapon.reduceWeaponWastageRate (2);
				}

				RFGameManager.instance.addExperience (2 * enemy.level + GameAttribute.gameAttribute.loadPlayerLevel () + 5);

				if (enemy.getIsStanding() && RFGameManager.instance.getGameState () == RFGameState.idle) {
					RFGameManager.instance.gameStartFighting ();
					enemy.resetResist ();
					enemy.setLookState (HeadLookState.none, null, Vector3.zero);
					enemy.setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
				}

				//gameManager.createBloodSplash(cp.collider2.gameObject, cp.cp);
			} else {
				if (weapon == null) {
				} else {
					weapon.playAttackSounds (false);
					weapon.reduceWeaponWastageRate (1);
				}
			}*/
			feedback.isHit = true;
		}
	}

	override protected void ResponseHitLimbsFeedback(HitFeedback feedback, ContactPointInfo cp){
		base.ResponseHitLimbsFeedback(feedback, cp);

		float volume = 0.03f * feedback.force.magnitude;
		if(feedback.feedbackARB == controller.getLFoot()){
			if(cp.collider2.tag.Contains("carpet")){
				lFootSoundSource.clip = bipedalSounds[0];
				lFootSoundSource.pitch = 1;
				lFootSoundSource.volume = volume;
				lFootSoundSource.Play();
			}else if(cp.collider2.tag.Contains("wood")){
				lFootSoundSource.clip = bipedalSounds[1];
				lFootSoundSource.pitch = 1;
				lFootSoundSource.volume = volume;
				lFootSoundSource.Play();
			}else if(cp.collider2.tag.Contains("terrain")){
				lFootSoundSource.clip = bipedalSounds[20];
				lFootSoundSource.pitch = 0.6f;
				lFootSoundSource.volume = volume;
				lFootSoundSource.Play();
			}
			/*if (lShoes != null && rShoes != null) {
				lShoes.reduceShoesWastageRate (0.0025f * feedback.force.magnitude);
				rShoes.reduceShoesWastageRate (0.0025f * feedback.force.magnitude);
			}*/
		}else if(feedback.feedbackARB == controller.getRFoot()){
			if(cp.collider2.tag.Contains("carpet")){
				rFootSoundSource.clip = bipedalSounds[0];
				lFootSoundSource.pitch = 1;
				rFootSoundSource.volume = volume;
				rFootSoundSource.Play();
			}else if(cp.collider2.tag.Contains("wood")){
				rFootSoundSource.clip = bipedalSounds[1];
				lFootSoundSource.pitch = 1;
				rFootSoundSource.volume = volume;
				rFootSoundSource.Play();
			}else if(cp.collider2.tag.Contains("terrain")){
				lFootSoundSource.clip = bipedalSounds[20];
				lFootSoundSource.pitch = 0.6f;
				lFootSoundSource.volume = volume;
				lFootSoundSource.Play();
			}
			/*if (lShoes != null && rShoes != null) {
				lShoes.reduceShoesWastageRate (0.0025f * feedback.force.magnitude);
				rShoes.reduceShoesWastageRate (0.0025f * feedback.force.magnitude);
			}*/
		}
	}
	
	IEnumerator moveToStand(){
		moveLevel = 1;
		yield return new WaitForSeconds(0.2f);
		startStand();
		moveLevel = 0;
	}
	
	void Update(){
		if(!characterIsReady) return;
		/*
		if(Input.GetKeyDown(KeyCode.Z)){
			GameObject target = getClosestPickerTarget(torso.transform.position, PickerDirection.none);
			if(target != null){
				Picker picker = target.GetComponent("Picker") as Picker;
				picker.updateLimbsEnable(true, true, false, false);
				picker.updateClosestToPickerPosition(controller.getTorso(), PickerDirection.none, character.getRoot(), PickerDirection.none);
				startReach(target);
			}
		}*/
		/*
		if (targetEnemys.Count == 0) {
			targetEnemy = null;
		} else if (targetEnemys.Count == 1) {
			targetEnemy = targetEnemys [0] as EnemyBip;
		} else {
			EnemyBip nearestEnemy = targetEnemys [0] as EnemyBip;
			foreach (EnemyBip enemy in targetEnemys) {
				if (nearestEnemy != enemy) {
					if ((pelvis.transform.position - nearestEnemy.pelvis.transform.position).magnitude > (pelvis.transform.position - enemy.pelvis.transform.position).magnitude) {
						nearestEnemy = enemy;
					}
				}
			}
			targetChangeTimer++;
			if (targetEnemy != nearestEnemy && targetChangeTimer > 200 && Random.Range (0, 100) < 2) {
				targetEnemy = nearestEnemy;
				targetChangeTimer = 0;
			}
		}

		if (RFGameManager.instance.getGameState () == RFGameState.fighting) {
			if (targetEnemy != null) {
				if (targetEnemyArea == ColliderArea.nearArea) {
					if (!isFallDown && !isHurting) {
						setLookState (HeadLookState.lookTarget, targetEnemy.head, Vector3.zero);
					} else {
						if (lookState == HeadLookState.lookTarget) {
							setLookState (HeadLookState.none, null, Vector3.zero);
						}
					}
					if (!isMoveing) {
						if (!isFighting) {
							setDirectionWithTargetPosition (targetEnemy.pelvis.transform.position);
						}

						Vector3 ptoe = targetEnemy.pelvis.transform.position - pelvis.transform.position;
						Vector3 axis = (Vector3.Project (ptoe, Vector3.right) + Vector3.Project (ptoe, Vector3.forward)).normalized;

						float ang = Vector3.Angle (axis, Vector3.forward);
						if (Vector3.Dot (axis, Vector3.right) < 0) {
							ang = -ang;
						}
						cameraControler.originRotation.x = ang - (cameraDirSign * 90);
						
						if (cameraControler.CurrentDistance < 10) {
							if (cameraChangeTimer == 0) {
								cameraDirSign = -cameraDirSign;
							}
							cameraChangeTimer += 1;
							//if(cameraChangeTimer > 200){
							//cameraChangeTimer = 0;
							//}
						} else {
							cameraChangeTimer = 0;
						}
					}
				} else if (targetEnemyArea == ColliderArea.farArea) {
					if (lookState == HeadLookState.lookTarget) {
						setLookState (HeadLookState.none, null, Vector3.zero);
					}
					cameraChangeTimer = 0;
					Vector3 ptoe = targetEnemy.pelvis.transform.position - pelvis.transform.position;
					Vector3 axis = (Vector3.Project (ptoe, Vector3.right) + Vector3.Project (ptoe, Vector3.forward)).normalized;

					float ang = Vector3.Angle (axis, Vector3.forward);
					if (Vector3.Dot (axis, Vector3.right) < 0) {
						ang = -ang;
					}
					cameraControler.originRotation.x = ang;
				}
			}
		} else if (RFGameManager.instance.getGameState () == RFGameState.idle) {
			if (!isHurting && !isDragging && isStanding) {
				if (isVigilance) {
					if (handTouchState == HandTouchState.none) {
						setLookState (HeadLookState.lookTarget, currCamera.gameObject, Vector3.zero);
						setDirectionWithTargetPosition (currCamera.gameObject.transform.position);
						if (getCurrentStateID () != (int)BipedalActions.stand_defense) {
							startDefenseStand ();
						}
						vigilanceTimer += 50 * Time.deltaTime;
						if (vigilanceTimer >= Random.Range (200, 300)) {
							vigilanceTimer = 0;
							isVigilance = false;
							setLookState (HeadLookState.none, null, Vector3.zero);
						}
					} else {
						vigilanceTimer = 0;
						isVigilance = false;
					}
				}

				if (RFGameManager.instance.getBalls ().Count > 0) {
					currBall = RFGameManager.instance.ballIsCasting ();
					if (currBall == null) {
						currBall = RFGameManager.instance.ballIsHolding ();
					}
					if (currBall == null && balls.Count > 0) {
						currBall = balls [0] as RubberBall;
						foreach (RubberBall ball in balls) {
							if (ball.ballState == BallState.drag) {
								currBall = ball;
								break;
							} else if (ball.ballState == BallState.normal) {
								if (currBall != ball) {
									if ((pelvis.transform.position - currBall.transform.position).magnitude > (pelvis.transform.position - ball.transform.position).magnitude) {
										currBall = ball;
									}
								}
							}
						}
					}

					if (currBall != null) {
						if (currBall.ballState == BallState.hold && canLookObject (currBall.gameObject)) {
							setLookState (HeadLookState.lookTarget, currBall.gameObject, Vector3.zero);
							setDirectionWithTargetPosition (currBall.gameObject.transform.position);
							if (getCurrentStateID () != (int)BipedalActions.stand_defense) {
								startDefenseStand ();
							}
							resetResist ();
						} else if ((currBall.ballState == BallState.cast) && canLookObject (currBall.gameObject)) {
							behaviour.bodySpringKp = 800;
							behaviour.bodySpringKd = 20;
							behaviour.belenceError = 0.15f;
							setLookState (HeadLookState.lookTarget, currBall.gameObject, Vector3.zero);
							setHandTouchState (HandTouchState.hit, currBall.gameObject, Vector3.zero, HandTouchMode.autoDoubleHand);
						} else if (currBall.ballState == BallState.drag) {
							if (currBall.getDragMoveSpeed () > 0.1f) {
								ballMoveTimer = 0;
								if (canLookObject (currBall.gameObject)) {
									behaviour.bodySpringKp = 600;
									behaviour.bodySpringKd = 25;
									behaviour.belenceError = 0.15f;
									setLookState (HeadLookState.lookTarget, currBall.gameObject, Vector3.zero);
									setHandTouchState (HandTouchState.resist, currBall.gameObject, Vector3.zero, HandTouchMode.autoDoubleHand);
								}
							} else {
								ballMoveTimer += 50 * Time.deltaTime;
								if (ballMoveTimer > Random.Range(60, 200)) {
									if (lookState == HeadLookState.lookTarget) {
										resetResist ();
										setLookState (HeadLookState.none, null, Vector3.zero);
										setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
									}
								}
							}
						} else if (currBall.ballState == BallState.normal) {
							if (lookState == HeadLookState.lookTarget) {
								resetResist ();
								setLookState (HeadLookState.none, null, Vector3.zero);
								setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
							}
						}
					} else {
						if (lookState == HeadLookState.lookTarget) {
							resetResist ();
							setLookState (HeadLookState.none, null, Vector3.zero);
							setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
						}
					}
				} else {
					if (lookState == HeadLookState.lookTarget) {
						resetResist ();
						setLookState (HeadLookState.none, null, Vector3.zero);
						setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
					}
				}
			}
		}*/
		
		if(Input.GetKeyDown(KeyCode.P)){
			string dataPath = Application.dataPath + "/Data/tempState.txt";
			controller.getCurrentState().writeStateToFile(dataPath, 0);
		}
	}

	override public void startHurt(HitFeedback feedback){
		if(isDead){
			startFallDown();
			//headSoundSource.clip = bipedalSounds[19];
			//headSoundSource.Play();
		}else{
			base.startHurt(feedback);
			//headSoundSource.clip = bipedalSounds[Random.Range(15, 19)];
			//headSoundSource.Play();
		}
	}
	/*
	public void removeTargetEnemy(EnemyBip target){
		if (targetEnemys.Contains (target)) {
			targetEnemys.Remove(target);
		}
	}
	
	public void addTargetEnemy(EnemyBip target){
		if (!targetEnemys.Contains (target)) {
			targetEnemys.Add(target);
		}
	}
*/
	/*
	public void addBall(RubberBall ball){
		if (!balls.Contains (ball)) {
			balls.Add (ball);
		}
	}
	public void removeBall(RubberBall ball){
		if (currBall == ball) {
			currBall = null;
		}
		if (balls.Contains (ball)) {
			balls.Remove(ball);
		}
	}
	public void clearBalls(){
		currBall = null;
		balls.Clear ();
	}
	*/
	public ArrayList getTargetEnemys(){
		return targetEnemys;
	}
	
	public ColliderArea getTargetEnemyArea(){
		return targetEnemyArea;
	}
	public void setTargetEnemyArea(ColliderArea area){
		targetEnemyArea = area;
	}
	
	public void startPunch(){
		if (!isFighting) {
			controller.setIsFreeHeading (true);
			//setLookState(HeadLookState.none, null, Vector3.zero);
			setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
			//Debug.Log("==== start punch ==== "+punchID);
		//	headSoundSource.clip = bipedalSounds [Random.Range (10, 15)];
		//	headSoundSource.Play ();
			controller.gotoState (punchID);
			punchID++;
			if (punchID > (int)BipedalActions.punch4) {
				punchID = (int)BipedalActions.punch1;
			}
		}
	}
		/*
	public void startWeaponPunch(){
		if (!isFighting) {
			weapon.startAttack ();
			controller.setIsFreeHeading (true);
			//setLookState(HeadLookState.none, null, Vector3.zero);
			setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
			//Debug.Log("==== start punch ==== "+punchID);
			headSoundSource.clip = bipedalSounds [Random.Range (10, 15)];
			headSoundSource.Play ();
			controller.gotoState (weaponPunchID);
			weaponPunchID++;
			if (weaponPunchID > (int)BipedalActions.weaponHit3) {
				weaponPunchID = (int)BipedalActions.weaponHit1;
			}
		}
	}*/
	
	public void startKick(bool onStand){
		if(!isFighting){
			controller.setIsFreeHeading (true);
			//setLookState(HeadLookState.none, null, Vector3.zero);
			setHandTouchState (HandTouchState.none, null, Vector3.zero, HandTouchMode.doubleHand);
		//	headSoundSource.clip = bipedalSounds [Random.Range (10, 15)];
		//	headSoundSource.Play ();
			controller.gotoState (kickID, onStand);
			kickID++;
			if (kickID > (int)BipedalActions.kick2) {
				kickID = (int)BipedalActions.kick1;
			}
		}
	}
	/*
	public void showPrompt(bool show, string txt = ""){
		if (show) {
			prompt.text = txt;
			prompt.gameObject.FadeTo (1, 0.3f, 0);
		} else {
			prompt.gameObject.FadeTo (0, 0.3f, 0);
		}
	}*/
}