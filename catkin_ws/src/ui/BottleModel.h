#ifndef BOTTLEMODEL_H
#define BOTTLEMODEL_H

#include <QAbstractListModel>
#include <QHash>


class Bottle
{
public:
    Bottle(float progress = 0.0, const QString &type = "hello", const QString &size = "there") {
        m_progress = progress;
        m_type = type;
        m_size = size;
    }

    QString type() const;
    void setType(const QString &type);

    QString size() const;
    void setSize(const QString &size);

    float progress() const;
    void setProgress(float progress);

private:
    float m_progress;
    QString m_type;
    QString m_size;
};

class BottleModel : public QAbstractListModel
{
    Q_OBJECT
public:
    enum BottleRoles {
        ProgressRole = Qt::UserRole + 1,
        TypeRole,
        SizeRole
    };

    BottleModel(QObject *parent = 0);

    void addBottle(Bottle bottle) {
        m_list.append(bottle);
    }

    QHash<int, QByteArray> roleNames() const {
        QHash<int, QByteArray> roles;
        roles[ProgressRole] = "progress";
        roles[TypeRole] = "type";
        roles[SizeRole] = "size";
        return roles;
    }

    int rowCount(const QModelIndex& parent = QModelIndex()) const {
        Q_UNUSED(parent)
        return m_list.size();
    }
    QVariant data(const QModelIndex& index, int role) const {
        if(!index.isValid()) {
                return QVariant();
            }
            if(index.row() < 0 || index.row() >= m_list.size()) {
                return QVariant();
            }
            if(role == TypeRole) {
                return QVariant(m_list.at(index.row()).type());
            }
            if(role == SizeRole) {
                return QVariant(m_list.at(index.row()).size());
            }
            if(role == ProgressRole) {
                return QVariant(m_list.at(index.row()).progress());
            }
            return QVariant();
    }

protected:
    QList<Bottle> m_list;
};


#endif // BOTTLEMODEL_H
