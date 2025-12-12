#include "serveranddrone.h"
#include <QDebug>

Link::Link(Server *n1,Server *n2,const QPair<Vector2D,Vector2D> &edge):
    node1(n1),node2(n2) {
    // computation of the length of the link
    Vector2D center=0.5*(edge.first+edge.second);
    distance = (center-Vector2D(n1->position.x(),n1->position.y())).length();
    distance += (center-Vector2D(n2->position.x(),n2->position.y())).length();
    edgeCenter=QPointF(center.x,center.y);
}

void Link::draw(QPainter &painter) {
    painter.drawLine(node1->position,edgeCenter);
    painter.drawLine(node2->position,edgeCenter);
}

/* Motions of the drone to reach the "destination" position*/
void Drone::move(qreal dt) {
    Vector2D dir=destination-position;
    double d=dir.length();

    if (d<slowDownDistance) {
        speed=(d*speedLocal/slowDownDistance)*dir;
    } else {
        speed+=(accelation*dt/d)*dir;
        if (speed.length()>speedMax) {
            speed.normalize();
            speed*=speedMax;
        }
    }
    // new position and orientation of the drone
    position+=(dt*speed);
    Vector2D Vn = (1.0/speed.length())*speed;
    if (Vn.y==0) {
        if (Vn.x>0) {
            azimut = -90;
        } else {
            azimut = 90.0;
        }
    } else if (Vn.y>0) {
        azimut = 180.0-180.0*atan(Vn.x/Vn.y)/M_PI;
    } else {
        azimut = -180.0*atan(Vn.x/Vn.y)/M_PI;
    }

    /* Write here your code that manages drone trajectories */
}

Server* Drone::overflownArea(QList<Server>& list) {
    auto it=list.begin();
    while (it!=list.end() && !it->area.contains(position)) {
        it++;
    }
    connectedTo= it!=list.end()?&(*it):nullptr;
    return connectedTo;
}
