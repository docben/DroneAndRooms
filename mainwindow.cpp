#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <canvas.h>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <trianglemesh.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // load initial simple case
    loadJson("../../json/simple.json");
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::loadJson(const QString& title) {
    QFile file(title);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Impossible d'ouvrir le fichier:" << title;
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "Erreur JSON:" << error.errorString();
        return false;
    }
    if (!doc.isObject()) {
        qWarning() << "Le document JSON n'est pas un objet.";
        return false;
    }

    QJsonObject root = doc.object();

    // --- Window ---
    if (root.contains("window") && root["window"].isObject()) {
        QJsonObject win = root["window"].toObject();

        auto origin = win.value("origine").toString().split(",");
        auto size   = win.value("size").toString().split(",");
        QPoint wOrigin={origin[0].toInt(),origin[1].toInt()};
        QSize wSize={size[0].toInt(),size[1].toInt()};
        qDebug() << "Window.origine =" << wOrigin;
        qDebug() << "Window.size    =" << wSize;
        ui->canvas->setWindow(wOrigin,wSize);
    }

    // --- Servers ---
    if (root.contains("servers") && root["servers"].isArray()) {
        int num=0;
        QJsonArray arr = root["servers"].toArray();
        for (const QJsonValue &v : arr) {
            if (!v.isObject()) continue;
            QJsonObject obj = v.toObject();
            Server s;
            s.name = obj.value("name").toString();
            QString pos = obj.value("position").toString();
            auto parts = pos.split(',');
            if (parts.size() == 2)
                s.position = QPoint(parts[0].toInt(), parts[1].toInt());
            s.color = QColor(obj.value("color").toString());
            s.id=num++;
            ui->canvas->servers.append(s);
            qDebug() << "Server:" << s.id << "," << s.name << s.position << s.color;
        }
    }

    // --- Drones ---
    if (root.contains("drones") && root["drones"].isArray()) {
        QJsonArray arr = root["drones"].toArray();

        for (const QJsonValue &v : arr) {
            if (!v.isObject()) continue;
            QJsonObject obj = v.toObject();
           Drone d;
            d.name = obj.value("name").toString();
            QString pos = obj.value("position").toString();
            auto parts = pos.split(',');
            if (parts.size() == 2)
                d.position = Vector2D(parts[0].toInt(), parts[1].toInt());
            QString name = obj.value("target").toString();
            // search name in server list
            d.target=nullptr;
            auto it=ui->canvas->servers.begin();
            while (it!=ui->canvas->servers.end() && it->name!=name) it++;
            if (it!=ui->canvas->servers.end()) {
                d.target=&(*it);
                qDebug() << "Drone:" << d.name << "(" << d.position.x << "," << d.position.y << ") →" << d.target->name;
            } else {
                qDebug() << "error in JsonFile: bad destination name: " << name;
            }
            ui->canvas->drones.append(d);
        }
    }

    createVoronoiMap();
    createServersLinks();
    fillDistanceArray();
    return true;
}

void MainWindow::createVoronoiMap() {
    TriangleMesh mesh(ui->canvas->servers);
    mesh.setBox(ui->canvas->getOrigin(),ui->canvas->getSize());

    auto triangles = mesh.getTriangles();
    auto m_servor = ui->canvas->servers.begin();
    QVector<const Triangle*> tabTri;
    while (m_servor!=ui->canvas->servers.end()) {
        // for all vertices of the mesh
        const Vector2D vert((*m_servor).position.x(),(*m_servor).position.y());
        auto mt_it = triangles->begin();
        tabTri.clear(); // tabTri: list of triangles containing m_vert
        while (mt_it!=triangles->end()) {
            if ((*mt_it).hasVertex(vert)) {
                tabTri.push_back(&(*mt_it));
            }
            mt_it++;
        }
        // find left border
        auto first = tabTri.begin();
        auto tt_it = tabTri.begin();
        bool found=false;
        while (tt_it!=tabTri.end() && !found) {
            auto comp_it = tabTri.begin();
            while (comp_it!=tabTri.end() && (*tt_it)->getNextVertex(vert)!=(*comp_it)->getPrevVertex(vert)) {
                comp_it++;
            }
            if (comp_it==tabTri.end()) {
                first=tt_it;
                found=true;
            }
            tt_it++;
        }
        // create polygon

        //poly->setColor((*m_servor)->color);
        tt_it=first;
        if (found && mesh.isInWindow((*tt_it)->getCenter().x,(*tt_it)->getCenter().y)) { // add a point for the left border
            Vector2D V = (*first)->nextEdgeNormal(vert);
            float k;
            if (V.x > 0) { // (circumCenter+k V).x=width
                k = (mesh.getWindowXmax() - (*first)->getCenter().x) / float(V.x);
            } else {
                k = (mesh.getWindowXmin()-(*first)->getCenter().x) / float(V.x);
            }
            if (V.y > 0) { // (circumCenter+k V).y=height
                k = fmin(k, (mesh.getWindowYmax() - (*first)->getCenter().y) / float(V.y));
            } else {
                k = fmin(k, (mesh.getWindowYmin()-(*first)->getCenter().y) / float(V.y));
            }
            m_servor->area.addVertex(Vector2D((*first)->getCenter() + k * V));
            Vector2D pt = (*first)->getCenter() + k * V;
        }
        auto comp_it = first;
        do {
            m_servor->area.addVertex((*tt_it)->getCenter());
            // search triangle on right of tt_it
            comp_it = tabTri.begin();
            while (comp_it!=tabTri.end() && (*tt_it)->getPrevVertex(vert)!=(*comp_it)->getNextVertex(vert)) {
                comp_it++;
            }
            if (comp_it!=tabTri.end()) tt_it = comp_it;
        } while (tt_it!=first && comp_it!=tabTri.end());
        if (found && mesh.isInWindow((*tt_it)->getCenter())) { // add a point for the right border
            Vector2D V = (*tt_it)->previousEdgeNormal(vert);
            float k;
            if (V.x > 0) { // (circumCenter+k V).x=width
                k = (mesh.getWindowXmax() - (*tt_it)->getCenter().x) / float(V.x);
            } else {
                k = (mesh.getWindowXmin()-(*tt_it)->getCenter().x) / float(V.x);
            }
            if (V.y > 0) { // (circumCenter+k V).y=height
                k = fmin(k, (mesh.getWindowYmax() - (*tt_it)->getCenter().y) / float(V.y));
            } else {
                k = fmin(k, (mesh.getWindowYmin()-(*tt_it)->getCenter().y) / float(V.y));
            }
            m_servor->area.addVertex(Vector2D((*tt_it)->getCenter() + k * V));
            Vector2D pt = (*tt_it)->getCenter() + k * V;
        }
        m_servor->area.clip(mesh.getWindowXmin(),mesh.getWindowYmin(),mesh.getWindowXmax(),mesh.getWindowYmax());
        m_servor->area.triangulate();

        m_servor++;
    }
}

void MainWindow::createServersLinks() {
    // for each polygon, if it exists a common edge with another
    // polygon, add a link between the servers.


}

void MainWindow::fillDistanceArray() {
    // define a nServers x nServers array
    int nServers = ui->canvas->servers.size();
    distanceArray.resize(nServers);
    for (int i=0; i<nServers; i++) {
        distanceArray[i].resize(nServers);
    }
    // init Servers distanceArray
    for (auto &s:ui->canvas->servers) {
        s.bestDistance.resize(nServers);
        for (int i=0; i<nServers; i++) {
            s.bestDistance[i]={nullptr,0};
        }
    }

    /* Write here the code to compute the distance array for all servers */

    // set first step Destinations
}

void MainWindow::update() {
    static int last=elapsedTimer.elapsed();
    int current=elapsedTimer.elapsed();
    int dt=current-last;
    // update positions of drones
    for (auto &drone:ui->canvas->drones) {
        drone.move(dt/1000.0);
    }
    ui->canvas->repaint();
}

void MainWindow::on_actionShow_graph_triggered(bool checked) {
    ui->canvas->showGraph=checked;
    ui->canvas->repaint();
}


void MainWindow::on_actionMove_drones_triggered() {
    timer = new QTimer(this);
    timer->setInterval(100);
    connect(timer,SIGNAL(timeout()),this,SLOT(update()));
    timer->start();

    elapsedTimer.start();
}


void MainWindow::on_actionQuit_triggered() {
    QApplication::quit();
}


void MainWindow::on_actionCredits_triggered() {
    QMessageBox::information(this,"About DroneAndRooms program",
                             "My tiny project.\nCredit Benoît Piranda");
}

