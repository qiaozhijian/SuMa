#include "KITTIReader.h"

#include <rv/FileUtil.h>
#include <algorithm>
#include <cmath>
#include <fstream>

#include <glow/glutil.h>
#include <rv/XmlDocument.h>
#include <rv/string_utils.h>
#include <boost/lexical_cast.hpp>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>

// TODO: read calibration from provided calibration files.

namespace rv {

KITTIReader::KITTIReader(const std::string& scan_filename, uint32_t buffer_size)
    : currentScan(0), bufferedScans(buffer_size), firstBufferedScan(0) {
    initScanFilenames(scan_filename);
}

void KITTIReader::initScanFilenames(const std::string& scan_filename) {
    scan_filenames.clear();

    std::vector<std::string> files = FileUtil::getDirectoryListing(FileUtil::dirName(scan_filename));//获取目录下文件指针数组
    /** filter irrelevant files. **/
    for (uint32_t i = 0; i < files.size(); ++i) {
        if (FileUtil::extension(files[i]) == ".pcd") {  //似乎拓展名是.bin也能用这个函数读
            scan_filenames.push_back(files[i]);
        }
    }

    std::sort(scan_filenames.begin(), scan_filenames.end());//对scan_filenames栈内文件进行升序排序
}

void KITTIReader::reset() {
    currentScan = 0;
    bufferedScans.clear();
    firstBufferedScan = 0;
}

bool KITTIReader::read(Laserscan& scan) {
    bool result = false;
    scan.clear();

    if (currentScan >= (int32_t)scan_filenames.size()) {//scan_filenames.size bin文件个数
        return false;
    }

    if (currentScan - firstBufferedScan < bufferedScans.size()) /** scan already in buffer, no need to read scan. **/
    {
        scan = bufferedScans[currentScan - firstBufferedScan];

        result = true;
    } else {
        result = read(currentScan, scan);//扫描不在buffer中，读取扫描
        if (result) {
            if (bufferedScans.capacity() == bufferedScans.size()) ++firstBufferedScan;
            bufferedScans.push_back(scan);
        }
    }

    ++currentScan;

    return result;
}

bool KITTIReader::isSeekable() const {
    return true;
}

void KITTIReader::seek(uint32_t scannr) {
    assert(scannr < scan_filenames.size());

    /** scan already in buffer, nothing to read just set current scan **/
    if (scannr - firstBufferedScan < bufferedScans.size()) {
        currentScan = scannr;
    } else if (currentScan < (int32_t)scannr) {//正常情况，scannr为需要读入的文件数，currentScan为当前文件
        /** if we don't have to read everything again than read missing scans. **/
        if ((scannr - 1) - currentScan < bufferedScans.capacity()) {
            currentScan =
                firstBufferedScan + bufferedScans.size(); /** advance to last scan in buffer to read no scans twice. **/
            while (currentScan < (int32_t)scannr) {
                Laserscan scan;

                if (bufferedScans.capacity() == bufferedScans.size()) ++firstBufferedScan;
                read(currentScan, scan);
                bufferedScans.push_back(scan);

                ++currentScan;
            }
        } else /** otherwise we just reset the buffer and start buffering again. **/
        {
            currentScan = scannr;
            firstBufferedScan = scannr;
            bufferedScans.clear();
        }
    } else if (currentScan > (int32_t)scannr) /** we have to add scans at the beginning **/
    {
        /** if we don't have to read every thing new, than read missing scans. **/
        if (currentScan - scannr < bufferedScans.capacity()) {
            currentScan = firstBufferedScan;

            while (currentScan > (int32_t)scannr) {
                --currentScan;

                Laserscan scan;

                read(currentScan, scan);
                bufferedScans.push_front(scan);
            }

            firstBufferedScan = currentScan;
        } else /** otherwise we just reset the buffer and start buffering again. **/
        {
            currentScan = scannr;
            firstBufferedScan = scannr;
            bufferedScans.clear();
        }
    }
}

uint32_t KITTIReader::count() const {
    return scan_filenames.size();
}

bool KITTIReader::read(uint32_t scan_idx, Laserscan& scan) {
    if (scan_idx > scan_filenames.size()) return false;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    //读取点云文件第scan_idx个文件的数据
    if ( pcl::io::loadPCDFile (scan_filenames[scan_idx].c_str(), cloud) ==-1 )
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }

    scan.clear();
    std::vector<Point3f>& points = scan.points_;
    std::vector<float>& remissions = scan.remissions_;
    int num_points = cloud.points.size();//num_points为点云个数
    points.resize(num_points);
    remissions.resize(num_points);

    float max_remission = 0;
    for (uint32_t i=0; i < num_points; i++) {
        points[i].x() = cloud.points[i].x;
        points[i].y() = cloud.points[i].y;
        points[i].z() = cloud.points[i].z;
        remissions[i] = cloud.points[i].intensity;
        max_remission = std::max(remissions[i], max_remission);
    }

    for (uint32_t i = 0; i < num_points; ++i) {
        remissions[i] /= max_remission;//对remissions数组归一化
    }

    return true;
}
// bool KITTIReader::read(uint32_t scan_idx, Laserscan& scan) {
//     if (scan_idx > scan_filenames.size()) return false;
//     std::ifstream in(scan_filenames[scan_idx].c_str(), std::ios::binary);
//     if (!in.is_open()) return false;
//
//     scan.clear();
//
//     in.seekg(0, std::ios::end);
//     uint32_t num_points = in.tellg() / (4 * sizeof(float));
//     in.seekg(0, std::ios::beg);
//
//     std::vector<float> values(4 * num_points);
//     in.read((char*)&values[0], 4 * num_points * sizeof(float));
//
//     in.close();
//     std::vector<Point3f>& points = scan.points_;
//     std::vector<float>& remissions = scan.remissions_;
//
//     points.resize(num_points);
//     remissions.resize(num_points);
//
//     float max_remission = 0;
//
//     for (uint32_t i = 0; i < num_points; ++i) {
//         points[i].x() = values[4 * i];
//         points[i].y() = values[4 * i + 1];
//         points[i].z() = values[4 * i + 2];
//         remissions[i] = values[4 * i + 3];
//         max_remission = std::max(remissions[i], max_remission);
//     }
//
//     for (uint32_t i = 0; i < num_points; ++i) {
//         remissions[i] /= max_remission;
//     }
//
//     return true;
// }

}
