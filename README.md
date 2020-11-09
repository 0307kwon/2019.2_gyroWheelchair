2019.2_gyroWheelchair
========================
2019년 2학기에 개발한 자이로 휠체어
# 0. index

1. [summary](https://github.com/0307kwon/2019.2_gyroWheelchair#1-summary)

2. [Theory](https://github.com/0307kwon/2019.2_gyroWheelchair#2-theory)

3. [하드웨어 구성](https://github.com/0307kwon/2019.2_gyroWheelchair#3-%ED%95%98%EB%93%9C%EC%9B%A8%EC%96%B4-%EA%B5%AC%EC%84%B1)

4. [코드 구조](https://github.com/0307kwon/2019.2_gyroWheelchair#4-%EC%BD%94%EB%93%9C-%EA%B5%AC%EC%A1%B0)

5. [기타](https://github.com/0307kwon/2019.2_gyroWheelchair#5-%EA%B8%B0%ED%83%80)


# 1. summary
<image src="readme_gif.gif" width=250px>
  
  
▲ 개발 결과


<image src="image07.jpg" width=400px>
  
▲ 휠체어와 조종기


휠체어가 오르막을 오를 때 차체가 기울어져 불안정한 자세가 형성된다.


자이로 휠체어는 이를 막기 위해 오르막에 의해 기울어지는 방향과 반대로 의자를 기울여


의자가 항상 같은 자세를 유지할 수 있도록 하며 안정감을 준다.



# 2. Theory
* thread(쓰레드)


  하나의 프로세스(프로그램) 내에서 여러가지의 함수(동작)들이 각각 cpu 자원을 잠깐 점유하다가


  다른 함수에게 내어주는 프로그래밍을 할 수가 있는데 이 속도는 매우 빠르므로


  마치 여러개의 함수가 동시에 실행되는 듯한 결과를 볼 수 있다.


  여기서 각각의 함수를 쓰레드라고 한다.


* priority(우선순위)


  각각의 쓰레드에는 priority(우선순위)가 주어진다.
  
  
  * 우선순위가 높은 쓰레드
  
  
    낮은 우선순위의 쓰레드가 실행되고 있어도 필요로 하면 강제로 cpu 자원을 점유할 수 있다.
  
  
  * 같은 우선순위인 쓰레드들
  
  
    round robin(동일한 시간 동안 점유) 등의 방식을 사용해 동작한다.
  
  
* mutex(뮤텍스)


  쓰레드가 동시에 실행되는 듯해도 결국 순차적인 코드 명령의 나열이다.


  이때 문제가 생기는데, 하나의 함수가 실행중이다가 주도권을 뺏겨 다른 함수를 실행시키고 다시 원래 함수의 동작으로 돌아왔을 때


  다른 함수에 의해 원래 함수가 사용하던 변수가 값이 변경되었을 수 있다. 이는 원치 않는 동작을 유발시키는 치명적인 상황이다.


  이를 막기위해 뮤텍스라는 개념이 도입되었다.



# 3. 하드웨어 구성
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
  
# 4. 코드 구조
<h2> 4-1. <a href="./최종코드/wheelchair.c">wheelchair.c</a> (차체)
  <h3>● 쓰레드 구조</h3>
    <image src="image03.png" width=900px>
  <h3>● 뮤텍스 구조</h3>
    <image src="image04.png" width=900px>  
<h2> 4-1. <a href="./최종코드/controller.c">controller.c</a> (조종기)</h2>
  <h3>● 쓰레드 구조</h3>
    <image src="image05.png" width=900px>
  <h3>● 뮤텍스 구조</h3>
    <image src="image06.png" width=900px>  

# 5. 기타

<div><image src="readme_image.png" width=600px><p><a href="https://sites.google.com/view/knu-rtlab/lectures?authuser=0">▲ 홈페이지에 게재된 자이로 휠체어</a></p></div>
