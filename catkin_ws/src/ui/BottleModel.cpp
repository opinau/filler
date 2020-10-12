#include "BottleModel.h"

BottleModel::BottleModel(QObject *parent) : QAbstractListModel(parent)
{
    m_list = QList<Bottle>();
}

QString Bottle::type() const
{
    return m_type;
}

void Bottle::setType(const QString &type)
{
    m_type = type;
}

QString Bottle::size() const
{
    return m_size;
}

void Bottle::setSize(const QString &size)
{
    m_size = size;
}

float Bottle::progress() const
{
    return m_progress;
}

void Bottle::setProgress(float progress)
{
    m_progress = progress;
}
