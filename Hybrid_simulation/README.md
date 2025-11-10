Questa cartella è dedicata al funzionamento corretto dell'*Hybrid Ultrasound Simulation* di **ImFusion** per generare le us sweep sintetiche da dare in pasto alla rete neurale per la sua segmentazione.

Per ottenere i corretti dati di input che l'algoritmo richiede, ho lavorato e testato due metodi. 
Il primo metodo prevede di ottenere tramite codice le due spline GlSline che richiede come input l'algoritmo e settare successivamente tutti i parametri della simulazione ultrasound "manualmente" tramite GUI.
Il secondo metodo prevede di generare direttamente l'*Utrasound Sweep* dalla tracking sequence per rispettare gli input indicati dalla documentazione (*https://docs.imfusion.com/suite/Ultrasound/Algorithms/US.UltrasoundSimulationHybrid.html*). 
# Registration Moveit!frames -> ImFusion frames
La tracking sequence *us_poses_1762452771_imfusion_frame.csv* è stata precedentemente registrata tramite il codice *convert_to_imfusion_frame.py* per allineare ambiente **moveit!** dal quale è stto ricavato il csv e ambiente **ImFusion**.
# Method 1: from tracking sequence to two spline
L'idea è quella di partire dalla tracking sequence ricavata dal file .csv, migliorre la traittoria "allisciandola" se troppo a zig zag e generare due spline:
- Transducer spline
- Direction spline
Queste due spline vengono generate come PolySpline, non adatte all'algritm di Hybrid Ultrasound simulation. Quindi serve modificare successivamente il workspace da:
```
<property name="GlPolyLine">
	<param name="editable">1</param>
	<param name="color">1 1 0 1 </param>
	<param name="lineWidth">1</param>
	<param name="labelVisible">1</param>
	<param name="labelBackgroundVisible">0</param>
	<param name="labelBackgroundColor">0.3 0.3 0.3 0.7 </param>
	<param name="labelBackgroundMargin">3</param>
	<param name="labelDepthTest">1</param>
	<param name="labelColor">0 1 1 </param>
	<param name="labelText">909.1</param>
	<param name="name">Polyline</param>
	<param name="points">245.030746459961 -181.163219928741 -182.627990722656 
210.030746459961 -42.163219928741 -182.627990722656 
377.030746459961 -118.163219928741 -182.627990722656 
164.030746459961 -154.163219928741 -182.627990722656 
324.030746459961 -34.163219928741 -182.627990722656 
245.030746459961 -178.163219928741 -182.627990722656 
245.030746459961 -180.163219928741 -182.627990722656 
</param>
	<param name="poseLinked">0</param>
</property>
```
a:
```
<property name="GlSpline">
	<param name="editable">1</param>
	<param name="color">1 1 0 1 </param>
	<param name="lineWidth">1</param>
	<param name="labelVisible">1</param>
	<param name="labelBackgroundVisible">0</param>
	<param name="labelBackgroundColor">0.3 0.3 0.3 0.7 </param>
	<param name="labelBackgroundMargin">3</param>
	<param name="labelDepthTest">1</param>
	<param name="labelColor">0 1 1 </param>
	<param name="labelText">909.1</param>
	<param name="name">Polyline</param>
	<param name="points">245.030746459961 -181.163219928741 -182.627990722656 
210.030746459961 -42.163219928741 -182.627990722656 
377.030746459961 -118.163219928741 -182.627990722656 
164.030746459961 -154.163219928741 -182.627990722656 
324.030746459961 -34.163219928741 -182.627990722656 
245.030746459961 -178.163219928741 -182.627990722656 
245.030746459961 -180.163219928741 -182.627990722656 
</param>

<param name="labelPixelOffset">7.82771971445489 -74.0752003653169 </param>
<param name="isClosed">0</param>
<param name="renderMode2d">0</param>
<param name="renderMode3d">0</param>
<param name="tubeThickness">1</param>
<param name="tubeEndT">1</param>
<param name="xrayTubeInnerRadius">0.7</param>

<param name="poseLinked">0</param>
</property>
```
per entrambe le spline.
Ecco un esempio di workspace corretto:
```
<?xml version="1.0" encoding="utf-8"?>
<propertyfile version="1.1" name="" comment="Written by ImFusionLib, 2025-11-08 14:30:07">
	<param name="workspaceVersion">29</param>
	<property name="Datasets">
		<property name="Data">
			<param name="uid">data0</param>
			<param name="name">Segmentation_decimated_better_points_mm</param>
		</property>
		<property name="Data">
			<param name="topDown">1</param>
			<param name="filename">../src/iiwa_probe_utils/Hybrid_simulation/segm_relabel.nii.gz</param>
			<param name="name">segm_relabel</param>
			<param name="original">1</param>
			<param name="transformation">-1 -1.22464679914735e-16 0 2.03074645996097 1.22464679914735e-16 -1 0 -126.663219928741 0 0 1 186.377990722656 0 0 0 1 </param>
			<param name="uid">data1</param>
			<property name="Components">
				<property name="DisplayOptions2d">
					<param name="windowingInitialized">1</param>
					<property name="TransferFunction">
						<param name="window">12</param>
						<param name="level">7</param>
						<param name="mode">0</param>
					</property>
				</property>
				<property name="DisplayOptions3d">
					<property name="TransferFunction">
						<param name="window">2774</param>
						<param name="level">363</param>
						<param name="presetName">CT Bone</param>
						<param name="mode">0</param>
						<property name="Tissues">
							<property name="Tissue">
								<property name="KeyPoint">
									<param name="position">0.4</param>
									<param name="color">0.752941176470588 0.250980392156863 0.125490196078431 0 </param>
								</property>
								<property name="KeyPoint">
									<param name="position">0.475</param>
									<param name="color">0.975 0.9 0.82 0.5 </param>
								</property>
								<property name="KeyPoint">
									<param name="position">0.5</param>
									<param name="color">1 1 1 0.6 </param>
								</property>
								<property name="KeyPoint">
									<param name="position">1</param>
									<param name="color">0.9 0.9 0.9 0.6 </param>
								</property>
							</property>
						</property>
					</property>
				</property>
				<property name="LabelDataComponent">
					<property name="LabelConfig">
						<param name="pixelValue">0</param>
						<param name="snomedCategoryCodeValue"/>
						<param name="snomedCategoryCodeMeaning"/>
						<param name="snomedTypeCodeValue"/>
						<param name="snomedTypeCodeMeaning"/>
					</property>
					<property name="LabelConfig">
						<param name="pixelValue">2</param>
						<param name="snomedCategoryCodeValue"/>
						<param name="snomedCategoryCodeMeaning"/>
						<param name="snomedTypeCodeValue"/>
						<param name="snomedTypeCodeMeaning"/>
					</property>
					<property name="LabelConfig">
						<param name="pixelValue">5</param>
						<param name="snomedCategoryCodeValue"/>
						<param name="snomedCategoryCodeMeaning"/>
						<param name="snomedTypeCodeValue"/>
						<param name="snomedTypeCodeMeaning"/>
					</property>
					<property name="LabelConfig">
						<param name="pixelValue">7</param>
						<param name="snomedCategoryCodeValue"/>
						<param name="snomedCategoryCodeMeaning"/>
						<param name="snomedTypeCodeValue"/>
						<param name="snomedTypeCodeMeaning"/>
					</property>
					<property name="LabelConfig">
						<param name="pixelValue">11</param>
						<param name="snomedCategoryCodeValue"/>
						<param name="snomedCategoryCodeMeaning"/>
						<param name="snomedTypeCodeValue"/>
						<param name="snomedTypeCodeMeaning"/>
					</property>
					<property name="LabelConfig">
						<param name="pixelValue">12</param>
						<param name="snomedCategoryCodeValue"/>
						<param name="snomedCategoryCodeMeaning"/>
						<param name="snomedTypeCodeValue"/>
						<param name="snomedTypeCodeMeaning"/>
					</property>
				</property>
			</property>
		</property>
	</property>
	<property name="Interface">
		<param name="outputUids">"data0" "data1" </param>
		<param name="inputUids"/>
	</property>
	<property name="Algorithms">
		<property name="IO.PointCloudIo">
			<param name="location">../src/iiwa_probe_utils/Hybrid_simulation/Segmentation_decimated_better_points_mm.pcd</param>
			<param name="saveBinary">0</param>
			<param name="saveColors">0</param>
			<param name="saveLabels">0</param>
			<param name="execute">1</param>
			<param name="inputUids"/>
			<param name="outputUids">"data0" </param>
		</property>
		<property name="IO.NiftiIo">
			<param name="location">../src/iiwa_probe_utils/Hybrid_simulation/segm_relabel.nii.gz</param>
			<param name="execute">1</param>
			<param name="inputUids"/>
			<param name="outputUids">"data1" </param>
		</property>
		<property name="Base.SetModality">
			<param name="modality">8</param>
			<param name="resetDisplayOptions">0</param>
			<param name="execute">1</param>
			<param name="inputUids">"data1" </param>
			<param name="outputUids"/>
		</property>
		<property name="Delete Data">
			<param name="execute">1</param>
			<param name="inputUids">"2" </param>
			<param name="outputUids">"data2" </param>
		</property>
		<property name="US.UltrasoundSimulationHybrid">
			<param name="attenuationCoefficient">0.1</param>
			<param name="gain">0</param>
			<param name="depth">50</param>
			<param name="elementsHeight">2</param>
			<param name="soundSpeed">1540</param>
			<param name="signalFreq">2</param>
			<param name="noiseSizeScale">4</param>
			<param name="noiseIntensityScale">1</param>
			<param name="waveWidth">2</param>
			<param name="SxFactor">6</param>
			<param name="SyFactor">3</param>
			<param name="wavePerPulse">2</param>
			<param name="intensityThreshold">9.99999997475243e-07</param>
			<param name="RFNoise">1e-05</param>
			<param name="TGCScaleFactor">0.01</param>
			<param name="TGCAlpha">0.5</param>
			<param name="rejectThreshold">0</param>
			<param name="scaleExponent1">1</param>
			<param name="scaleExponent2">0.5</param>
			<param name="focusDepth">40</param>
			<param name="reflectFactor">1</param>
			<param name="hilbertLength">8</param>
			<param name="elevationRayCount">10</param>
			<param name="linesCount">128</param>
			<param name="axialSamplePoints">512</param>
			<param name="rayCount">1280</param>
			<param name="multReflection">0</param>
			<param name="beamSmoothness">6</param>
			<param name="convolutionParallelRuns">2</param>
			<param name="sliceFlipX">0</param>
			<param name="sliceFlipY">0</param>
			<param name="dataFlipX">0</param>
			<param name="dataFlipY">0</param>
			<param name="inputUids">"data1" </param>
			<param name="outputUids"/>
			<property name="SyntheticUltrasoundSweepAlgorithm">
				<param name="frameCount">40</param>
				<param name="frameGeometry">1</param>
				<param name="sweepDuration">50</param>
				<param name="pixelsY">512</param>
			</property>
			<property name="AcousticParameters">
				<property name="1_Background">
					<param name="SoundSpeed_m_s">1540</param>
					<param name="AcousticImpedance_g_cm2s">150000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.200000002980232</param>
					<param name="Speckle_m0">0</param>
					<param name="Speckle_m1">0</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="2_Lung">
					<param name="SoundSpeed_m_s">1300</param>
					<param name="AcousticImpedance_g_cm2s">143000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.639999985694885</param>
					<param name="Speckle_m0">0.5</param>
					<param name="Speckle_m1">0.5</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="3_Fat">
					<param name="SoundSpeed_m_s">1470</param>
					<param name="AcousticImpedance_g_cm2s">142000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.479999989271164</param>
					<param name="Speckle_m0">0.5</param>
					<param name="Speckle_m1">0.5</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="4_Water">
					<param name="SoundSpeed_m_s">1492</param>
					<param name="AcousticImpedance_g_cm2s">149000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.0199999995529652</param>
					<param name="Speckle_m0">0</param>
					<param name="Speckle_m1">0</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="5_CSF">
					<param name="SoundSpeed_m_s">1515</param>
					<param name="AcousticImpedance_g_cm2s">152000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.0020000000949949</param>
					<param name="Speckle_m0">0.0500000007450581</param>
					<param name="Speckle_m1">0.00999999977648258</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="6_Kidney">
					<param name="SoundSpeed_m_s">1540</param>
					<param name="AcousticImpedance_g_cm2s">106000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.200000002980232</param>
					<param name="Speckle_m0">0.490000009536743</param>
					<param name="Speckle_m1">0.200000002980232</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="7_Blood">
					<param name="SoundSpeed_m_s">1492</param>
					<param name="AcousticImpedance_g_cm2s">149000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.0199999995529652</param>
					<param name="Speckle_m0">0</param>
					<param name="Speckle_m1">0</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="8_Muscle">
					<param name="SoundSpeed_m_s">1568</param>
					<param name="AcousticImpedance_g_cm2s">163000</param>
					<param name="AttenuationCoeff_dB_MHzcm">1.0900000333786</param>
					<param name="Speckle_m0">0.529999971389771</param>
					<param name="Speckle_m1">0.509999990463257</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="9_Grey Matter">
					<param name="SoundSpeed_m_s">1590</param>
					<param name="AcousticImpedance_g_cm2s">30000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.540000021457672</param>
					<param name="Speckle_m0">0.300000011920929</param>
					<param name="Speckle_m1">0.200000002980232</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="10_White Matter">
					<param name="SoundSpeed_m_s">1530</param>
					<param name="AcousticImpedance_g_cm2s">80000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.540000021457672</param>
					<param name="Speckle_m0">0.5</param>
					<param name="Speckle_m1">0.349999994039536</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="11_Liver">
					<param name="SoundSpeed_m_s">1540</param>
					<param name="AcousticImpedance_g_cm2s">106000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.200000002980232</param>
					<param name="Speckle_m0">0.490000009536743</param>
					<param name="Speckle_m1">0.400000005960464</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="12_Soft Tissue">
					<param name="SoundSpeed_m_s">1540</param>
					<param name="AcousticImpedance_g_cm2s">163000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.540000021457672</param>
					<param name="Speckle_m0">0.529999971389771</param>
					<param name="Speckle_m1">0.5</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="13_Bone">
					<param name="SoundSpeed_m_s">3600</param>
					<param name="AcousticImpedance_g_cm2s">612000</param>
					<param name="AttenuationCoeff_dB_MHzcm">7.80000019073486</param>
					<param name="Speckle_m0">0.779999971389771</param>
					<param name="Speckle_m1">0.560000002384186</param>
					<param name="Speckle_s0">0.100000001490116</param>
				</property>
				<property name="14_Skull">
					<param name="SoundSpeed_m_s">3600</param>
					<param name="AcousticImpedance_g_cm2s">612000</param>
					<param name="AttenuationCoeff_dB_MHzcm">7.80000019073486</param>
					<param name="Speckle_m0">0.779999971389771</param>
					<param name="Speckle_m1">0.560000002384186</param>
					<param name="Speckle_s0">0.100000001490116</param>
				</property>
				<property name="15_Vessel">
					<param name="SoundSpeed_m_s">1540</param>
					<param name="AcousticImpedance_g_cm2s">106000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.5</param>
					<param name="Speckle_m0">0.899999976158142</param>
					<param name="Speckle_m1">0.649999976158142</param>
					<param name="Speckle_s0">0.239999994635582</param>
				</property>
				<property name="16_Brain Tumor">
					<param name="SoundSpeed_m_s">1530</param>
					<param name="AcousticImpedance_g_cm2s">163000</param>
					<param name="AttenuationCoeff_dB_MHzcm">0.699999988079071</param>
					<param name="Speckle_m0">0.800000011920929</param>
					<param name="Speckle_m1">0.5</param>
					<param name="Speckle_s0">0</param>
				</property>
				<property name="17_Air">
					<param name="SoundSpeed_m_s">345</param>
					<param name="AcousticImpedance_g_cm2s">4.09999990463257</param>
					<param name="AttenuationCoeff_dB_MHzcm">1.63999998569489</param>
					<param name="Speckle_m0">0</param>
					<param name="Speckle_m1">0</param>
					<param name="Speckle_s0">0</param>
				</property>
			</property>
		</property>
	</property>
	<property name="Annotations">
		<property name="GlPointCloud">
			<param name="editable">1</param>
			<param name="color">1 1 0 1 </param>
			<param name="lineWidth">1</param>
			<param name="labelVisible">1</param>
			<param name="labelBackgroundVisible">0</param>
			<param name="labelBackgroundColor">0.3 0.3 0.3 0.7 </param>
			<param name="labelBackgroundMargin">3</param>
			<param name="labelDepthTest">1</param>
			<param name="labelColor">0 1 1 </param>
			<param name="labelText"/>
			<param name="name">Segmentation_decimated_better_points_mm</param>
			<param name="pointSize">2</param>
			<param name="drawLines">0</param>
			<param name="drawNormals">0</param>
			<param name="colorWithNormals">0</param>
			<param name="poseLinked">0</param>
			<param name="referenceDataUid">data0</param>
		</property>
		<property name="GlSpline">
			<param name="editable">1</param>
			<param name="color">0.937254901960784 0.16078431372549 0.16078431372549 1 </param>
			<param name="lineWidth">1</param>
			<param name="labelVisible">1</param>
			<param name="labelBackgroundVisible">0</param>
			<param name="labelBackgroundColor">0.3 0.3 0.3 0.7 </param>
			<param name="labelBackgroundMargin">3</param>
			<param name="labelDepthTest">1</param>
			<param name="labelColor">0 1 1 </param>
			<param name="labelText">155.1 mm</param>
			<param name="name">Transducer Spline</param>
			<param name="points">9.58328225000001 -25.13086225 -118.875849 
 9.8824447162203 -25.3625910277685 -120.396639301153 
 10.3146857202599 -25.5853740498263 -121.886182813364 
 10.6565240355933 -25.8324288739046 -123.394952067246 
 10.9213839378295 -26.0981535097322 -124.916047584249 
 11.0804419645622 -26.3828067696129 -126.447180675591 
 11.1021111174369 -26.6887757917737 -127.982398539613 
 11.1069452335782 -27.0374602146048 -129.509469410618 
 11.1827258568203 -27.4285555147645 -131.024598086769 
 11.2270879638021 -27.7772552335719 -132.55142404386 
 11.2201562717996 -28.1034222525938 -134.084249321363 
 11.1864185549722 -28.4141285527581 -135.619527194857 
 11.1381851239861 -28.7191374679301 -137.15566693755 
 11.1442129898669 -29.0286921632473 -138.691767829715 
 11.2044031967256 -29.3408470521368 -140.225992418867 
 11.3168390399227 -29.6538647973527 -141.757321890267 
 11.4226623200778 -29.9430771806568 -143.293730323704 
 11.5126344157243 -30.2128397845529 -144.834867472241 
 11.5820382621221 -30.4622993227617 -146.380309175045 
 11.6367176518011 -30.6960837951776 -147.928968135901 
 11.693958975392 -30.932260332355 -149.47715929173 
 11.7390532093169 -31.181704995318 -151.023491642904 
 11.7451327140434 -31.4443789318298 -152.568465443599 
 11.7511047085262 -31.7216830064544 -154.110887672821 
 11.7074462229598 -32.01246274477 -155.650210240245 
 11.6053719962382 -32.3071988513329 -157.18575132554 
 11.5575748378446 -32.5643939164458 -158.730029445202 
 11.4983335854085 -32.8053122390148 -160.277002523931 
 11.460539975036 -33.0167798006137 -161.828265689675 
 11.4763304372214 -33.1941597042799 -163.385251989327 
 11.4806038962816 -33.3717272442987 -164.942189907945 
 11.4699544884043 -33.5522075661417 -166.498866814589 
 11.4505674449679 -33.7311040925895 -168.05554964538 
 11.4282532607379 -33.9158890871207 -169.61141425074 
 11.4498345387997 -34.0870605736716 -171.168954703314 
 11.5027583468 -34.2394351191997 -172.72766332304 
 11.6029138398698 -34.3800922377041 -174.285261880294 
 11.7267010095335 -34.5014215598514 -175.842809786439 
 11.826441269132 -34.5923698186913 -177.404109203937 
 11.8579594851124 -34.7193032736266 -178.965420041644 
 11.8566774549358 -34.8337875221652 -180.528259027228 
 11.8078851532633 -34.9600564249523 -182.088996897251 
 11.7397912104444 -35.1107409997995 -183.647228067292 
 11.704994123826 -35.2668104447166 -185.206190389126 
 11.6551587317978 -35.4108036266619 -186.765928169965 
 11.6551632505146 -35.5638375859014 -188.325531227526 
 11.6433989993186 -35.7071100247327 -189.886050881768 
 11.6079734716894 -35.8499016116409 -191.446187360285 
 11.5592886402413 -35.9825556358897 -193.006849482177 
 11.5170617447603 -36.0891429395322 -194.56979525036 
 11.4410616232113 -36.1992467661984 -196.131237108579 
 11.3751476155907 -36.2803382274767 -197.694842589395 
 11.3021701873639 -36.3479232178166 -199.25880248722 
 11.2321740833097 -36.3999456792927 -200.823382410386 
 11.1865412827505 -36.4314355724378 -202.389543264839 
 11.1282765014798 -36.4438117793235 -203.955545604339 
 11.1093175416779 -36.4294024062198 -205.522527568349 
 11.1409785964508 -36.3769360512117 -207.088394858873 
 11.1807481784115 -36.3037909201098 -208.653333594142 
 11.2728473167692 -36.1895610744349 -210.213332307786 
 11.3762369196148 -36.0534559180826 -211.771134392789 
 11.3978987972053 -35.9567420176402 -213.334414977994 
 11.3188973007592 -35.8749391936426 -214.896833956403 
 11.1433438849175 -35.8314618907956 -216.452601253237 
 10.8771490749402 -35.8062563159235 -217.996152974777 
 10.5834070171808 -35.7622462721063 -219.534800983841 
 10.3383852950211 -35.6852584826558 -221.08014828434 
 10.1396436591637 -35.5736005530247 -222.630460028274 
 9.9893479233019 -35.4264696629764 -224.182919723533 
 9.85790961085464 -35.2698492052149 -225.736539735092 
 9.73535513393692 -35.1090671648665 -227.290202665899 
 9.63292047701421 -34.9416649195928 -228.844897792708 
 9.53609431467276 -34.770809860929 -230.399334736414 
 9.51051772396015 -34.5592375898292 -231.950883497268 
 9.52260099602121 -34.2889560422345 -233.494396192753 
 9.55842861150187 -33.9664405471632 -235.026184217029 
 9.63034199131257 -33.5834317274688 -236.543991556361 
 9.70226071551616 -33.1735725753137 -238.054707982853 
 9.76657004143766 -32.7285000730214 -239.555787205498 
 9.83640298456329 -32.2504526230496 -241.046423141 
 9.89538407989945 -31.7450571197882 -242.528550545583 
 9.93775048353846 -31.2289128131665 -244.007663334717 
 9.97698548216765 -30.7096353302559 -245.485773972777 
 10.0124457500959 -30.186001777819 -246.96243179175 
 10.0445189739032 -29.6584595492809 -248.437786004522 
 10.0437531589576 -29.1383092683513 -249.915929341107 
 10.0275893690344 -28.6216250153152 -251.39538047659 
 9.96399811836162 -28.1160397306737 -252.877310233221 
 9.89287494314545 -27.6104036711627 -254.35894297944 
 9.81117458409418 -27.1010100280997 -255.838754695659 
 9.71978264406757 -26.5879886753345 -257.316724924386 
 9.65050867788595 -26.0675580776499 -258.79212150633 
 9.70578689844372 -25.5224219366073 -260.260375726069 
 9.89478991854603 -24.9551438821659 -261.707260117392 
 10.12332256727 -24.3822969194912 -263.147954147212 
 10.2788672742376 -23.7922346516826 -264.589859199807 
 10.2424693269791 -23.1704862655446 -266.02590118872 
 9.96128826523936 -22.5230808271826 -267.424223285506 
 9.73621284147622 -21.8852284010747 -268.837510351435 
 9.55370600000003 -21.255304 -270.260838 
 </param>
			<param name="pointColor">0.937254901960784 0.16078431372549 0.16078431372549 1 </param>
			<param name="labelPixelOffset">1.46882062920319 -35.6921277124227 </param>
			<param name="isClosed">0</param>
			<param name="renderMode3d">0</param>
			<param name="parentDataUid">data1</param>
			<param name="poseLinked">0</param>
		</property>
		<property name="GlSpline">
			<param name="editable">1</param>
			<param name="color">0.541176470588235 0.886274509803922 0.203921568627451 1 </param>
			<param name="lineWidth">1</param>
			<param name="labelVisible">1</param>
			<param name="labelBackgroundVisible">0</param>
			<param name="labelBackgroundColor">0.3 0.3 0.3 0.7 </param>
			<param name="labelBackgroundMargin">3</param>
			<param name="labelDepthTest">1</param>
			<param name="labelColor">0 1 1 </param>
			<param name="labelText">155.1 mm</param>
			<param name="name">Direction Spline</param>
			<param name="points">9.58328225000001 -85.13086225 -118.875849 
 9.8824447162203 -85.3625910277685 -120.396639301153 
 10.3146857202599 -85.5853740498263 -121.886182813364 
 10.6565240355933 -85.8324288739046 -123.394952067246 
 10.9213839378295 -86.0981535097322 -124.916047584249 
 11.0804419645622 -86.3828067696129 -126.447180675591 
 11.1021111174369 -86.6887757917737 -127.982398539613 
 11.1069452335782 -87.0374602146048 -129.509469410618 
 11.1827258568203 -87.4285555147645 -131.024598086769 
 11.2270879638021 -87.7772552335719 -132.55142404386 
 11.2201562717996 -88.1034222525938 -134.084249321363 
 11.1864185549722 -88.4141285527581 -135.619527194857 
 11.1381851239861 -88.7191374679301 -137.15566693755 
 11.1442129898669 -89.0286921632473 -138.691767829715 
 11.2044031967256 -89.3408470521368 -140.225992418867 
 11.3168390399227 -89.6538647973527 -141.757321890267 
 11.4226623200778 -89.9430771806568 -143.293730323704 
 11.5126344157243 -90.2128397845529 -144.834867472241 
 11.5820382621221 -90.4622993227617 -146.380309175045 
 11.6367176518011 -90.6960837951776 -147.928968135901 
 11.693958975392 -90.932260332355 -149.47715929173 
 11.7390532093169 -91.181704995318 -151.023491642904 
 11.7451327140434 -91.4443789318298 -152.568465443599 
 11.7511047085262 -91.7216830064544 -154.110887672821 
 11.7074462229598 -92.01246274477 -155.650210240245 
 11.6053719962382 -92.3071988513329 -157.18575132554 
 11.5575748378446 -92.5643939164458 -158.730029445202 
 11.4983335854085 -92.8053122390148 -160.277002523931 
 11.460539975036 -93.0167798006137 -161.828265689675 
 11.4763304372214 -93.1941597042799 -163.385251989327 
 11.4806038962816 -93.3717272442987 -164.942189907945 
 11.4699544884043 -93.5522075661417 -166.498866814589 
 11.4505674449679 -93.7311040925895 -168.05554964538 
 11.4282532607379 -93.9158890871207 -169.61141425074 
 11.4498345387997 -94.0870605736716 -171.168954703314 
 11.5027583468 -94.2394351191997 -172.72766332304 
 11.6029138398698 -94.3800922377041 -174.285261880294 
 11.7267010095335 -94.5014215598514 -175.842809786439 
 11.826441269132 -94.5923698186913 -177.404109203937 
 11.8579594851124 -94.7193032736266 -178.965420041644 
 11.8566774549358 -94.8337875221652 -180.528259027228 
 11.8078851532633 -94.9600564249523 -182.088996897251 
 11.7397912104444 -95.1107409997995 -183.647228067292 
 11.704994123826 -95.2668104447166 -185.206190389126 
 11.6551587317978 -95.4108036266619 -186.765928169965 
 11.6551632505146 -95.5638375859014 -188.325531227526 
 11.6433989993186 -95.7071100247327 -189.886050881768 
 11.6079734716894 -95.8499016116409 -191.446187360285 
 11.5592886402413 -95.9825556358898 -193.006849482177 
 11.5170617447603 -96.0891429395322 -194.56979525036 
 11.4410616232113 -96.1992467661984 -196.131237108579 
 11.3751476155907 -96.2803382274767 -197.694842589395 
 11.3021701873639 -96.3479232178166 -199.25880248722 
 11.2321740833097 -96.3999456792927 -200.823382410386 
 11.1865412827505 -96.4314355724378 -202.389543264839 
 11.1282765014798 -96.4438117793235 -203.955545604339 
 11.1093175416779 -96.4294024062198 -205.522527568349 
 11.1409785964508 -96.3769360512117 -207.088394858873 
 11.1807481784115 -96.3037909201098 -208.653333594142 
 11.2728473167692 -96.1895610744349 -210.213332307786 
 11.3762369196148 -96.0534559180826 -211.771134392789 
 11.3978987972053 -95.9567420176402 -213.334414977994 
 11.3188973007592 -95.8749391936426 -214.896833956403 
 11.1433438849175 -95.8314618907956 -216.452601253237 
 10.8771490749402 -95.8062563159235 -217.996152974777 
 10.5834070171808 -95.7622462721063 -219.534800983841 
 10.3383852950211 -95.6852584826558 -221.08014828434 
 10.1396436591637 -95.5736005530247 -222.630460028274 
 9.9893479233019 -95.4264696629764 -224.182919723533 
 9.85790961085464 -95.2698492052149 -225.736539735092 
 9.73535513393692 -95.1090671648664 -227.290202665899 
 9.63292047701421 -94.9416649195928 -228.844897792708 
 9.53609431467276 -94.770809860929 -230.399334736414 
 9.51051772396015 -94.5592375898292 -231.950883497268 
 9.52260099602121 -94.2889560422345 -233.494396192753 
 9.55842861150187 -93.9664405471632 -235.026184217029 
 9.63034199131257 -93.5834317274688 -236.543991556361 
 9.70226071551616 -93.1735725753137 -238.054707982853 
 9.76657004143766 -92.7285000730214 -239.555787205498 
 9.83640298456329 -92.2504526230496 -241.046423141 
 9.89538407989945 -91.7450571197882 -242.528550545583 
 9.93775048353846 -91.2289128131665 -244.007663334717 
 9.97698548216765 -90.7096353302559 -245.485773972777 
 10.0124457500959 -90.186001777819 -246.96243179175 
 10.0445189739032 -89.6584595492809 -248.437786004522 
 10.0437531589576 -89.1383092683513 -249.915929341107 
 10.0275893690344 -88.6216250153152 -251.39538047659 
 9.96399811836162 -88.1160397306737 -252.877310233221 
 9.89287494314545 -87.6104036711627 -254.35894297944 
 9.81117458409418 -87.1010100280997 -255.838754695659 
 9.71978264406757 -86.5879886753345 -257.316724924386 
 9.65050867788595 -86.0675580776499 -258.79212150633 
 9.70578689844372 -85.5224219366073 -260.260375726069 
 9.89478991854603 -84.9551438821659 -261.707260117392 
 10.12332256727 -84.3822969194912 -263.147954147212 
 10.2788672742376 -83.7922346516826 -264.589859199807 
 10.2424693269791 -83.1704862655446 -266.02590118872 
 9.96128826523936 -82.5230808271826 -267.424223285506 
 9.73621284147622 -81.8852284010747 -268.837510351435 
 9.55370600000003 -81.255304 -270.260838 
 </param>
			<param name="pointColor">0.541176470588235 0.886274509803922 0.203921568627451 1 </param>
			<param name="labelPixelOffset">1.46882062920319 -35.6921277124229 </param>
			<param name="isClosed">0</param>
			<param name="renderMode3d">0</param>
			<param name="parentDataUid">data1</param>
			<param name="poseLinked">0</param>
		</property>
	</property>
	<property name="Display">
		<param name="layoutMode">0</param>
		<param name="maximisedView">4</param>
		<param name="focusedView">0</param>
		<param name="viewOrder">0 1 2 3 4 </param>
		<property name="Views">
			<property name="0">
				<param name="isVisible">0</param>
				<param name="flip">1</param>
				<param name="showZoom">1</param>
			</property>
			<property name="1">
				<param name="isVisible">0</param>
				<param name="flip">1</param>
				<param name="zoom">1.20321844012769</param>
				<param name="showZoom">1</param>
				<param name="viewMatrix">1 0 0 2.03074645996098 0 1 0 -126.663219928741 0 0 1 -186.377990722656 0 0 0 1 </param>
				<param name="sliceMatrix">1 0 0 2.03074645996098 0 1 0 -126.663219928741 0 0 1 -186.377990722656 0 0 0 1 </param>
			</property>
			<property name="2">
				<param name="isVisible">0</param>
				<param name="flip">1</param>
				<param name="zoom">1.20321844012769</param>
				<param name="showZoom">1</param>
				<param name="viewMatrix">0 0 -1 10.936996459961 1 0 0 -126.663219928741 0 -1 0 -186.377990722656 0 0 0 1 </param>
				<param name="sliceMatrix">0 0 -1 10.936996459961 1 0 0 -126.663219928741 0 -1 0 -186.377990722656 0 0 0 1 </param>
			</property>
			<property name="3">
				<param name="isVisible">0</param>
				<param name="flip">1</param>
				<param name="zoom">1.20321844012769</param>
				<param name="showZoom">1</param>
				<param name="viewMatrix">1 0 0 2.03074645996098 0 0 1 -126.663219928741 0 -1 0 -186.377990722656 0 0 0 1 </param>
				<param name="sliceMatrix">1 0 0 2.03074645996098 0 0 1 -126.663219928741 0 -1 0 -186.377990722656 0 0 0 1 </param>
			</property>
			<property name="4">
				<param name="isVisible">1</param>
				<param name="volumeRendererName">General Purpose Volume Renderer</param>
				<param name="sceneCenter">2.03074645996098 -126.663219928741 -186.377990722656 </param>
				<param name="ssaoMode">1</param>
				<property name="Camera">
					<param name="poseMatrix">-0.525495378515927 0.057503103268572 0.848850988261715 167.032449973882 0.606515031334507 0.72500256072462 0.3263599296912 151.262178988701 -0.59665243142361 0.68634151852939 -0.415861991553771 -801.208288252826 0 0 0 1 </param>
					<param name="focalLength">1.37373870972731 1.37373870972731 </param>
				</property>
				<property name="VolumeRenderer">
					<param name="maxVolumesToDisplay">4</param>
					<param name="mode">4</param>
					<param name="ambientAttenuationDistance">0</param>
				</property>
				<property name="SSAO">
					<param name="ssaoStrength">0.75</param>
					<param name="occlusionRadius">21</param>
				</property>
			</property>
		</property>
		<property name="VisibleData">
			<property name="0">
				<param name="visible">"2" </param>
			</property>
			<property name="1">
				<param name="visible">"data1" "2" </param>
			</property>
			<property name="2">
				<param name="visible">"data1" "2" </param>
			</property>
			<property name="3">
				<param name="visible">"data1" "2" </param>
			</property>
			<property name="4">
				<param name="visible">"data1" "2" </param>
			</property>
		</property>
	</property>
</propertyfile>
```
Esempio diretto è il workspace caricato **method_1.iws**
Una volta avuti come input le due spline e la segmentazione:
<img width="1845" height="859" alt="immagine" src="https://github.com/user-attachments/assets/995b54dd-f164-49a8-8aa4-6ade12dc2bfe" />
è possibile avviare la **Hybrid Ultrasound Simulation**.

# Method 2: from tracking sequence to ultrasound sweep
Creare un ultrasound sweep a partire dalla tracking sequence.
## Allisciare la tracking sequence e rendere la traiettoria più smooth:
All'interno della console python di Imfusion
```
import io; exec(io.open("/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/method_2/smooth_tracking_sequence.py", mode="r", encoding="utf-8").read(), globals())
```
## Generare un ultrasound sweep a partire dal tracking sequence:
questo codice è stato scritto in collborazione con un consulente di ImFusion stesso. Ho dovuto apportare delle modifiche alla sua base per adattarlo ale mie esigenze.
```
import sys, importlib; sys.path.append("/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation"); import sweep_ortho_from_tracking as so; importlib.reload(so); so.build_sweep_ortho()
```
