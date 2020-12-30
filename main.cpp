#include <iostream>
#include<thread>
#include<mutex>
#include<condition_variable>
#include<queue>

#include <vector>
#include<stdlib.h>
#include <math.h>

#define FREQ_CAM 30
#define FREQ_IMU 100
#define T0_CAM 0.0
#define T0_IMU 0.42
#define DURATION 5

using namespace std;

int j = 0;
int k = 0;

struct Q {
    double x;
    double y;
    double z;
    double w;
};

Q qc[150];

double rand1() {
    double i;
    i = (rand() % 100) / 100.0;
    return i;
};
Q NLerp(const Q& q1, const Q& q2, const double& t) {

    Q q;
    q.x = (1.0 - t) * q1.x + t * q2.x;
    q.y = (1.0 - t) * q1.y + t * q2.y;
    q.z = (1.0 - t) * q1.z + t * q2.z;
    q.w = (1.0 - t) * q1.w + t * q2.w;
    double sum;
    sum = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    q.x = q.x / sum;
    q.y = q.y / sum;
    q.z = q.z / sum;
    q.w = q.w / sum;
    return q;
};
Q Norm(const Q& q) {
    double sum;
    Q q1;
    sum = q.x + q.y + q.z + q.w;
    q1.x = q.x / sum;
    q1.y = q.y / sum;
    q1.z = q.z / sum;
    q1.w = q.w / sum;
    return q1;
};
void GenTimeStamp(std::vector<double>& cam_t, std::vector<double>& imu_t, std::vector<Q>& imu_q) {

    int j = 0;
    int i = 0;
    Q imuq;
    imuq.x = 0.0;
    imuq.y = 0.0;
    imuq.z = 0.0;
    imuq.w = 1.0;

    while (i * (1.0) / FREQ_CAM < 5) {
        cam_t.push_back(T0_CAM + i * (1.0 / FREQ_CAM) + 0.5 * (1.0 / FREQ_CAM) * rand1());

        i++;

    }
    while (j * (1.0) / FREQ_IMU < 5) {
        imu_t.push_back(T0_IMU + j * (1.0 / FREQ_IMU) + 0.5 * (1.0 / FREQ_IMU) * rand1());
        imuq.x = (imuq.x + 0.1 * rand1());
        imuq.y = (imuq.y + 0.1 * rand1());
        imuq.z = (imuq.z + 0.1 * rand1());
        imuq.w = (imuq.w + 0.1 * rand1());
        imuq = Norm(imuq);

        imu_q.push_back(imuq);

        j++;
    }

};

bool CalImuPose(const double& tc, const std::vector<double>& imu_t, const std::vector<Q>& imu_q) {

    bool a = false;
    Q q;
    if (tc == 0.0) {
        return a;
    }
    for (unsigned int i = 1; i < imu_t.size(); i++) {

        if (imu_t[i - 1] <= tc && imu_t[i] >= tc) {
            q = NLerp(imu_q[i - 1], imu_q[i], tc);
            qc[j].x = q.x;
            qc[j].y = q.y;
            qc[j].z = q.z;
            qc[j].w = q.w;
            j++;

            a = true;
            break;
        }
    }
    return  a;

};

//任务队列
queue< double>products;
queue< int>num;

mutex m;
condition_variable cond;
bool notify = false;
bool done = false;

void producer() ;
void consumer() {
    int k;
    while (!done) {

        //上锁保护共享资源,unique_lock一次实现上锁和解锁
        unique_lock<mutex>lk(m);
        //等待生产者者通知有资源
        while (!notify) {

            cond.wait(lk);
        }
        
        //要是队列不为空的话
        while (!products.empty()) {

            cout << "cam_t:" << products.front() << endl;
            products.pop();
            k = num.front();
            num.pop();
            cout << "q:(" << qc[k].x << "," << qc[k].y << "," << qc[k].z << "," << qc[k].w << ")" << endl;
            k++;
            //通知生产者仓库容量不足,生产产品
            notify = false;
            cond.notify_one();
        }
    }
}
vector<double> cam_t;
vector<double> imu_t;
vector<Q> imu_q;
void producer() {
    int i = 0;
    for (const auto& tc : cam_t) {

        if (CalImuPose(tc, imu_t, imu_q)) {
            unique_lock<mutex>lk(m);
            //cout << "producer..." <<  << endl;
            //如果仓库中有产品,就等待消费者消费完后在生产
            while (notify || !products.empty()) {
                cond.wait(lk);
            }
            //当前仓库里面没有东西了,就将产品装入仓库
            products.push(tc);
            num.push(i);
            i++;
            //设置有产品的通知
            notify = true;
            //通知消费者可以取产品了
            cond.notify_one();
            //cout << "(" << qc[i].x << "," << qc[i].y << "," << qc[i].z << "," << qc[i].w << ")" << endl;
            //i++;
        }
    }
    

    


    //通知消费者端不生产了
    done = true;
    cond.notify_one();
}

int main()
{
    GenTimeStamp(cam_t, imu_t, imu_q);
    thread t1(producer);
    thread t2(consumer);
    t1.join();
    t2.join();
    return 0;
}
