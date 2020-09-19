#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[],QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Set default brush size label
    ui->brushSize_label->setText("5");

    connect(ui->canvasWidget, &CanvasWidget::pointerMoved, this, &MainWindow::updatePosLabel);

    rrt = false;
    rrtStar = false;
    anytimeRRT = false;

    ui->stepSizeValue->setPlaceholderText("0 - 100");
    ui->stepSizeValue->setValidator(new QIntValidator(0,100,this));

    ui->maxIterValue->setPlaceholderText("any interger");
    ui->maxIterValue->setValidator(new QIntValidator(0,2147483647,this));

    ui->neighbourFactorValue->setPlaceholderText("0.0 - 10.0");
    ui->neighbourFactorValue->setValidator(new QDoubleValidator(0.0,10.0,1,this));

    ui->costToGoFactorValue->setPlaceholderText("0.0 - 25.0");
    ui->costToGoFactorValue->setValidator(new QDoubleValidator(0.0,25.0,1,this));

    ui->algoSpeedValue->setPlaceholderText("0 - 100000");
    ui->algoSpeedValue->setValidator(new QIntValidator(0,1000000,this));

    ui->maxRunsValue->setPlaceholderText("0 - 50");
    ui->maxRunsValue->setValidator(new QIntValidator(0,50,this));


    if(QString::compare(QString::fromUtf8(argv[1]),"rrt") == 0)
        rrt = true;
    else if(QString::compare(QString::fromUtf8(argv[1]),"rrtstar") == 0)
        rrtStar = true;
    else if(QString::compare(QString::fromUtf8(argv[1]),"anytimerrt") == 0)
        anytimeRRT = true;

    if(rrtStar){
        ui->costToGoFactor->setVisible(false);
        ui->costToGoFactorValue->setVisible(false);
        ui->algoSpeed->setVisible(false);
        ui->algoSpeedValue->setVisible(false);
        ui->maxRuns->setVisible(false);
        ui->maxRunsValue->setVisible(false);
        this->setWindowTitle("RRT Star");
    }
    else if(rrt){
        ui->neighbouFactor->setVisible(false);
        ui->neighbourFactorValue->setVisible(false);
        ui->costToGoFactor->setVisible(false);
        ui->costToGoFactorValue->setVisible(false);
        ui->algoSpeed->setVisible(false);
        ui->algoSpeedValue->setVisible(false);
        ui->maxRuns->setVisible(false);
        ui->maxRunsValue->setVisible(false);
        this->setWindowTitle("RRT");
    }
    else
        this->setWindowTitle("Anytime RRT");

    yamlData[0] = "image: map1.png";
    yamlData[1] = "resolution: 0.1";
    yamlData[2] = "origin: [0.0, 0.0, 0.0]";
    yamlData[3] = "occupied_thresh: 0.65";
    yamlData[4] = "free_thresh: 0.196";
    yamlData[5] = "negate: 0";

}

MainWindow::~MainWindow()
{
    delete ui;
}

//============================================================
//Slider methods
//============================================================
void MainWindow::on_brushSize_slider_sliderReleased()
{
    //Call appropriate method to set brush size
    ui->canvasWidget->setBrushSize(ui->brushSize_slider->value());
}

void MainWindow::on_brushSize_slider_sliderMoved(int position)
{
    ui->brushSize_label->setText(QString::number(position));
}

void MainWindow::on_brushSize_slider_valueChanged(int value)
{
    ui->brushSize_label->setText(QString::number(value));
    ui->canvasWidget->setBrushSize(value);
}

//============================================================
//Various methods
//============================================================
void MainWindow::on_clearAll_clicked()
{
    ui->canvasWidget->clearAll();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);

    changeSize();

    updatePosLabel();
}

void MainWindow::changeSize()
{

    QString width = QString::number(ui->canvasWidget->size().width());
    QString height = QString::number(ui->canvasWidget->size().height());

    ui->widthLabel->setText(width);
    ui->heightLabel->setText(height);
}

void MainWindow::updatePosLabel(){
    startX = QString::number(ui->canvasWidget->startPos.x());
    startY = QString::number(ui->canvasWidget->startPos.y());

    ui->startX->setText(startX);
    ui->startY->setText(startY);

    endX = QString::number(ui->canvasWidget->endPos.x());
    endY = QString::number(ui->canvasWidget->endPos.y());

    ui->endX->setText(endX);
    ui->endY->setText(endY);
}
void MainWindow::on_saveButton_clicked()
{
    QImage canvasImage = ui->canvasWidget->getImage();
    //Get filename path (don't forget to add suffix at the end! .jpg for example)
    QString filePath = QFileDialog::getSaveFileName(this, "SaveImage", "", "PNG (*.png);;JPEG (*.jpg *.jpeg)");
    //Check if path is null
    if (filePath == "")
    {
        qDebug() << "File was not saved";
        return;
    }
    //Save image
    canvasImage.save(filePath);
    yamlData[0] = "image: " + filePath;

    QString yamlFile,launchFile;
    if (anytimeRRT) {
        yamlFile = "../../anytimerrt/maps/map.yaml";
        launchFile = "../../anytimerrt/launch/anytimeRRT.launch";
    }
    else if(rrtStar){
        yamlFile = "../../rrtstar/maps/map.yaml";
        launchFile = "../../rrtstar/launch/rrtStar.launch";
    }
    else if(rrt){
        yamlFile = "../../rrt/maps/map.yaml";
        launchFile = "../../rrt/launch/rrt.launch";
    }
    qDebug() << yamlFile;
    QFile yaml(yamlFile);
    QFile launch(launchFile);
    if (!yaml.open(QFile::WriteOnly |
                   QFile::Text))
    {
        qDebug() << " Could not open the file for reading";
        return;
    }
    QTextStream yamlOut(&yaml);
    for(int i=0;i<6;i++){
        yamlOut << yamlData[i] << "\n";
    }
    yaml.close();
    qDebug() << "Finished writing yaml file";
    if (!launch.open(QFile::ReadOnly |
                   QFile::Text))
    {
        qDebug() << " Could not open the file for reading";
        return;
    }
    QTextStream in(&launch);
    int ctr = 0;
       while (!in.atEnd())
       {
          launchData[ctr++] = in.readLine();
          qDebug() << launchData[ctr-1];
       }
       launch.close();
    launchData[1] =   " <arg name = \"startX\" value=\"" + startX + "\"/>";
    launchData[2] =   " <arg name = \"startY\" value=\"" + startY + "\"/>";
    launchData[3] =   " <arg name = \"endX\" value=\"" + endX + "\"/>";
    launchData[4] =   " <arg name = \"endY\" value=\"" + endY + "\"/>";
    launchData[5] =   " <arg name = \"stepSize\" value=\"" + ui->stepSizeValue->text() + "\"/>";
    launchData[6] =   " <arg name = \"maxIter\" value=\"" + ui->maxIterValue->text() + "\"/>";
    if(rrtStar){
    launchData[7] =   " <arg name = \"neighbourFactor\" value=\"" + ui->neighbourFactorValue->text() + "\"/>";
    }
    else if(anytimeRRT){
    launchData[7] =   " <arg name = \"neighbourFactor\" value=\"" + ui->neighbourFactorValue->text() + "\"/>";
    launchData[8] =   " <arg name = \"costToGoFactor\" value=\"" + ui->costToGoFactorValue->text() + "\"/>";
    launchData[9] =   " <arg name = \"algoSpeed\" value=\"" + ui->algoSpeedValue->text() + "\"/>";
    launchData[10] =   " <arg name = \"maxRuns\" value=\"" + ui->maxRunsValue->text() + "\"/>";
    }
    if (!launch.open(QFile::WriteOnly |
                   QFile::Text))
    {
        qDebug() << " Could not open the file for reading";
        return;
    }
    QTextStream launchOut(&launch);
    for(int i=0;i<ctr;i++){
        launchOut << launchData[i] << "\n";
    }
    QCoreApplication::quit();

}
