2019.2_gyroWheelchair
========================
2019년 2학기에 개발한 자이로 휠체어
# 1. summary
<image src="readme_gif.gif" width=250px>
▲ 실험 결과

휠체어가 오르막을 오를 때 차체가 기울어져 불안정한 자세가 형성된다.  
자이로 휠체어는 이를 막기 위해 오르막에 의해 기울어지는 방향과 반대로 의자를 기울여  
의자가 항상 같은 자세를 유지할 수 있도록 하며 안정감을 준다.

# 2. 구성
1. 차체
<image src="image01.png" width=450px>  
<p>● IMU 센서를 통해 의자의 기울기를 인식</p>  
<p>● 인식한 기울기를 바탕으로 의자를 회전시킴</p>  
  <p>● 블루투스로 조종기와 통신</p>  
  
2. 조종기
<image src="image02.png" width=900px>
<p>● 컬러인식센서로 휠체어의 전진,정지,후진을 구현  </p> 
<p>● 모터의 엔코더를 이용해 핸들의 회전 방향을 인식하고 오른쪽 회전시 차체를 오른쪽으로 회전, 왼쪽으로 회전시 차체를 왼쪽으로 회전시킨다.  </p> 
<p>● 조종기와 차체와의 통신 방법으로 블루투스를 사용, 무선 조종이 가능함  </p> 
  
# 3. 구조
<h2> 3-1. <a href="./최종코드/wheelchair.c">wheelchair.c</a> (차체)
  <h3>● 쓰레드 구조</h3>
    <image src="image03.png" width=900px>
  <h3>● 뮤텍스 구조</h3>
    <image src="image04.png" width=900px>  
<h2> 3-1. <a href="./최종코드/controller.c">controller.c</a> (조종기)</h2>
  <h3>● 쓰레드 구조</h3>
    <image src="image03.png" width=900px>
  <h3>● 뮤텍스 구조</h3>
    <image src="image04.png" width=900px>  
# 4. 코드
  

# 5. 기타

<div><image src="readme_image.png" width=600px><p>▲ 홈페이지에 게재된 자이로 휠체어 </p></div>
