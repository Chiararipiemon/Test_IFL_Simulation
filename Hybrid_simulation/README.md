# Method 1: from tracking sequence to two spline
L'idea è quella di partire dalla tracking sequence ricavata dal file .csv, migliorre la trittoria "allisciandola" se troppo a zig zag e generare due spline:
- Transducer spline
- Direction spline
Queste due spline vengono generate come PolySpline, non adatte all'algritm di Hybrid Ultrasound simulation. Quindi serve modificare successivamente il workspace da:
'''
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
a:
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
'''
per entrambe le spline
Creare un ultrasound sweep a partire dalla tracking sequence: 
